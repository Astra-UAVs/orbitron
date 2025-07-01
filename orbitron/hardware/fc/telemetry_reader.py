from __future__ import annotations
import time
import dataclasses
import queue
import threading
import logging
from pymavlink import mavutil

@dataclasses.dataclass
class TelemetrySample:
    t_rx: float
    msg_name: str
    data: dict


class _BaseLoop:
    def __init__(
            self,
            connection: str,
            topics_hz: dict[str, int],
            out_q: queue.Queue|None=None,
            secret_key: bytes|None = None,
            link_id: int|None = None
        ):
        self._conn = connection
        self._topics = topics_hz
        self._queue = out_q or queue.Queue(maxsize=300)
        self._key = secret_key
        self._link_id = link_id
        self._stop_evt = threading.Event()

    def stop(self): self._stop_evt.set()
    def stopped(self): return self._stop_evt.is_set()

    def _open(self):
        link = mavutil.mavlink_connection(self._conn, autoreconnect=True, source_system=255)
        if self._key:
            link.setup_signing(
                secret_key=self._key,
                sign_outgoing=True,
                link_id=self._link_id
            )
        link.wait_heartbeat()
        return link
    
    def _request(self, link):
        for name, hz in self._topics.items():
            msgid = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}")
            link.mav.command_long_send(
                link.target_system,
                link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msgid,
                int(1_000_000/hz),
                0, 0, 0, 0, 0
            )

    def _loop(self):
        link = self._open()
        self._request(link)
        last_beat = 0.0

        while not self.stopped():
            if time.time() - last_beat > 1.0:
                link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                last_beat = time.time()

            msg = link.recv_match(blocking=True, timeout=0.5)
            if not msg or msg.get_type() == 'BAD_DATA':
                continue
            if msg.get_type() not in self._topics:
                continue

            sample = TelemetrySample(time.time(), msg.get_type(), msg.to_dict())
            try:
                self._queue.put_nowait(sample)
            except queue.Full:
                _ = self._queue.get_nowait()
                self._queue.put_nowait(sample)

        link.close()
        logging.info("%s stopped", self.__class__.__name__)

_ENV_HZ = {          # Hz per message
    'SCALED_IMU'      : 10,
    'SCALED_PRESSURE' :  5,
    'SCALED_PRESSURE2':  5,
    'SCALED_PRESSURE3':  5,
}

class EnvThread(threading.Thread, _BaseLoop):
    def __init__(self, **kw):
        threading.Thread.__init__(self, daemon=True)
        _BaseLoop.__init__(self, topics_hz=_ENV_HZ, **kw)
    def run(self): self._loop()

_ENERGY_HZ = {
    'BATTERY_STATUS':  2,   # slow-changing
    'POWER_STATUS'  :  2,
    'ESC_STATUS'    : 10,   # per-motor RPM & currents
}

class EnergyThread(threading.Thread, _BaseLoop):
    def __init__(self, **kw):
        threading.Thread.__init__(self, daemon=True)
        _BaseLoop.__init__(self, topics_hz=_ENERGY_HZ, **kw)
    def run(self): self._loop()

_FM_HZ = {
    'HEARTBEAT'                   : 1,
    'MISSION_CURRENT'             : 2,
    'NAV_CONTROLLER_OUTPUT'       : 5,
    'GIMBAL_DEVICE_ATTITUDE_STATUS':10,  # or 'MOUNT_STATUS'
}

class FlightMgmtThread(threading.Thread, _BaseLoop):
    def __init__(self, **kw):
        threading.Thread.__init__(self, daemon=True)
        _BaseLoop.__init__(self, topics_hz=_FM_HZ, **kw)
    def run(self): self._loop()

_RED_HZ = { 'TIMESYNC': 10 }          # request 10 Hz ping-pong

class RedundancyThread(threading.Thread, _BaseLoop):
    def __init__(self, **kw):
        threading.Thread.__init__(self, daemon=True)
        _BaseLoop.__init__(self, topics_hz=_RED_HZ, **kw)
    def run(self): self._loop()
