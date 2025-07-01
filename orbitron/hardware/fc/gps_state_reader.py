from __future__ import annotations
import time
import os
import queue
import pathlib
import dataclasses, threading
import logging
from typing import Any
from pymavlink import mavutil

@dataclasses.dataclass
class GpsSample:
    t_rx: float
    msg_name: str
    data: dict

def load_key(path: os.PathLike) -> bytes | None:
    p = pathlib.Path(path)
    return p.read_bytes() if p.exists() else None


class _GpsLoop:
    DEFAULT_HZ = {'GPS_RAW_INT': 5, 'GLOBAL_POSITION_INT': 10}

    def __init__(self, connection:str='udpin:0.0.0.0:14550', out_queue:Any=None, req_hz:dict[str, int] | None = None, secret_key:bytes|None = None, link_id: int|None = None):
        self._conn_str = connection
        self._queue = out_queue or queue.Queue(maxsize=200)
        self._req_hz = req_hz or self.DEFAULT_HZ
        self._secret_key = secret_key
        self._link_id = link_id
        self._stop_evt = threading.Event()

    def stop(self): self._stop_evt.set()
    def stopped(self): return self._stop_evt.is_set()

    def _open(self):
        link = mavutil.mavlink_connection(self._conn_str, autoreconnect=True, source_system=255)
        if self._secret_key:
            link.setup_signing(secret_key=self._secret_key, sign_outgoing=True, link_id=self._link_id)
            logging.info("Signing enabled (link id=%d)", link.mav.signing.link_id)
        link.wait_heartbeat()
        logging.info("Heartbeat OK – sys %d comp %d", link.target_system, link.target_component)
        return link
    
    def _request(self, link):
        for name, hz in self._req_hz.items():
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")
            interval = int(1_000_000 / hz)
            link.mav.command_long_send(link.target_system, link.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval, 0, 0, 0, 0, 0)

    def _loop(self):
        link = self._open()
        self._request(link)

        last_hb = 0.0
        while not self.stopped():
            if time.time() - last_hb > 1.0:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = time.time()

            msg = link.recv_match(blocking=True, timeout=0.5)
            if not msg or msg.get_type() == 'BAD_DATA':
                continue
            if msg.get_type() not in self._req_hz:
                continue

            sample = GpsSample(time.time(), msg.get_type(), msg.to_dict())
            try:
                self._queue.put_nowait(sample)
            except queue.Full:
                _ = self._queue.get_nowait()
                self._queue.put_nowait(sample)

        link.close()
        logging.info("GPS loop closed")

class GpsThread(threading.Thread, _GpsLoop):
    def __init__(self, **kw):
        threading.Thread.__init__(self, daemon=True)
        _GpsLoop.__init__(self, **kw)
    def run(self): self._loop()

# if __name__ == '__main__':
#     logging.basicConfig(level=logging.INFO,
#                         format='%(asctime)s %(levelname)s: %(message)s')

#     key = load_key(pathlib.Path.home() / '.telemetry_key')  # optional

#     q = mp.Queue()
#     gps = GpsThread(connection='udpin:localhost:14550',
#                     out_queue=q,
#                     secret_key=key).start()

#     try:
#         while True:
#             s: GpsSample = q.get()
#             if s.msg_name == 'GLOBAL_POSITION_INT':            # [5]
#                 lat = s.data['lat'] * 1e-7
#                 lon = s.data['lon'] * 1e-7
#                 alt = s.data['relative_alt'] / 1000
#                 print(f"{s.t_rx:.2f}  {lat:.6f},{lon:.6f}  alt {alt:.1f} m")
#             elif s.msg_name == 'GPS_RAW_INT':
#                 print(f"{s.t_rx:.2f}  Fix:{s.data['fix_type']}  "
#                       f"Sats:{s.data['satellites_visible']}")
#     except KeyboardInterrupt:
#         print("Stopping…")
#     finally:
#         gps.stop(); gps.join()