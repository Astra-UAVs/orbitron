from __future__ import annotations
import math
import pathlib
import threading
import time
from typing import Optional, Any
import queue
import dataclasses
import logging
import os

from pymavlink import mavutil

@dataclasses.dataclass
class NavSample:
    t_rx: float
    msg_id: int
    data: dict

def generate_secret_key(path: str | os.PathLike, overwrite: bool = False) -> bytes:
    path = pathlib.Path(path)
    if path.exists() and not overwrite:
        raise FileExistsError(path)
    key = os.urandom(32)
    path.write_bytes(key)
    return key

def load_secret_key(path: str | os.PathLike) -> bytes:
    key = pathlib.Path(path).read_bytes()
    if len(key) != 32:
        raise ValueError("Secret key must be 32 bytes")
    return key

class _MavLinkLoop:
    DEFAULT_MSGS = {
        'GLOBAL_POSITION_INT'  : 10,
        'ATTITUDE'             : 20,
        'GPS_RAW_INT'          :  5,
        'SYS_STATUS'           :  2,
        'VFR_HUD'              :  5,
    }

    def __init__(self, connection: str, out_queue:Any=None, requests: dict[str, int]=None, signing_key: bytes|None=None, sign_outgoing: bool=True, link_id: int=1, allow_unsigned: bool=False):
        self._cxn_str = connection
        self._queue = out_queue or queue.Queue(maxsize=200)
        self._requests = requests or self.DEFAULT_MSGS
        self._sign_key = signing_key
        self._sign_outgoing = sign_outgoing
        self._allow_unsigned_flag = allow_unsigned
        self._link_id = link_id
        self._stop_flag = threading.Event()

    def stop(self): self._stop_flag.set()
    def stopped(self): return self._stop_flag.is_set()

    @staticmethod
    def _unsigned_callback(self_, msg_id: int) -> bool:
        if msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_RADIO_STATUS, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT):
            return True
        return False

    def _make_connection(self):
        mlink = mavutil.mavlink_connection(self._cxn_str, autoreconnect=True, source_system=255)

        if self._sign_key:
            mlink.setup_signing(
                secret_key=self._sign_key,
                sign_outgoing=self._sign_outgoing,
                allow_unsigned_callback=(self._unsigned_callback if self._allow_unsigned_flag else None),
                link_id=self._link_id
            )
            logging.info("MAVLink 2 signing ENABLED (link_id=%s)", mlink.mav.signing.link_id)
        else:
            logging.info("MAVLink signing NOT enabled (no key provided)")

        mlink.wait_heartbeat()
        tgt_sys, tgt_comp = mlink.target_system, mlink.target_component
        logging.info("Connected: system %d component %d", tgt_sys, tgt_comp)
        return mlink
    
    def _request_streams(self, link):
        for name, hz in self._requests.items():
            try:
                msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")
            except AttributeError:
                logging.warning("Unknown MAVLink msg '%s' - skipping", name)
                continue
            interval = int(1_000_000 / hz)
            link.mav.command_long_send(
                link.target_system, link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id, interval, 0,0,0,0,0)
            logging.debug("Requested %s @ %d Hz", name, hz)

    def _loop(self):
        link = self._make_connection()
        self._request_streams(link)

        last_heartbeat = 0.0
        while not self.stopped():
            if time.time() - last_heartbeat > 1.0:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_heartbeat = time.time()

            msg = link.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == 'BAD_DATA':
                continue

            sample = NavSample(time.time(), msg.get_type(), msg.to_dict())
            try:
                self._queue.put_nowait(sample)
            except queue.Full:
                _ = self._queue.get_nowait()
                self._queue.put_nowait(sample)

        link.close()
        logging.info("Telemetry loop terminated.")


class MavTelemetryThread(threading.Thread, _MavLinkLoop):
    def __init__(self, **kw):
        threading.Thread.__init__(self, daemon=True)
        _MavLinkLoop.__init__(self, **kw)
    
    def run(self): self._loop()
            

# if __name__ == '__main__':
#     logging.basicConfig(format="%(asctime)s  %(levelname)s: %(message)s",
#                         level=logging.INFO)

#     # ---- secret key handling --------------------------------------
#     KEY_PATH = pathlib.Path.home() / ".telemetry_key"
#     if not KEY_PATH.exists():               # run once
#         generate_secret_key(KEY_PATH)
#     key_bytes = load_secret_key(KEY_PATH)

#     # ---- start telemetry thread -----------------------------------
#     q = mp.Queue()
#     telem = MavTelemetryThread(
#         connection='udpin:0.0.0.0:14550',
#         out_queue=q,
#         secret_key=key_bytes,
#         sign_outgoing=True,
#         allow_unsigned=True            # accept RADIO_STATUS/HEARTBEAT
#     )
#     telem.start()

#     try:
#         while True:
#             nav: NavSample = q.get()
#             if nav.msg_name == 'GLOBAL_POSITION_INT':
#                 lat = nav.payload['lat'] * 1e-7
#                 lon = nav.payload['lon'] * 1e-7
#                 alt = nav.payload['relative_alt'] / 1000.0
#                 print(f"{nav.t_rx:.2f}  {lat:.6f},{lon:.6f}  alt {alt:.1f} m")
#     except KeyboardInterrupt:
#         print("Ctrl-C – shutting down")
#     finally:
#         telem.stop(); telem.join()