import threading
import time
import can
from dataclasses import dataclass


NODE_ID = 1
CAN_CHANNEL = "can0"
CAN_BITRATE = 500000

COB_NMT = 0x000
COB_SDO_TX = 0x600 + NODE_ID
COB_SDO_RX = 0x580 + NODE_ID


class CanopenError(Exception):
    pass


@dataclass
class DriveConfig:
    mode: str = "VEL"       # VEL / POS
    posmode: str = "ABS"    # ABS / REL
    rpm: int = 200
    maxrpm: int = 7000
    acc: int = 10
    dec: int = 10
    pos: int = 0
    enabled: bool = False


class EposController:
    def __init__(self, channel="can0", node_id=1):
        self.node_id = node_id
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.cfg = DriveConfig()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.run_active = False
        self.telem_enabled = False
        self.telem_thread = None

    # =========================
    # CAN base
    # =========================
    def send_frame(self, arb_id: int, data: bytes):
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def recv_frame(self, expected_id: int, timeout=1.5):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.bus.recv(timeout=0.05)
            if msg is None:
                continue
            if msg.arbitration_id == expected_id:
                return msg
        raise CanopenError(f"TIMEOUT 0x{expected_id:X}")

    # =========================
    # SDO
    # =========================
    def sdo_write_u8(self, index, subindex, value):
        data = bytes([
            0x2F,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            value & 0xFF, 0, 0, 0
        ])
        with self.lock:
            self.send_frame(COB_SDO_TX, data)
            msg = self.recv_frame(COB_SDO_RX)
        if msg.data[0] == 0x60:
            return
        if msg.data[0] == 0x80:
            self._raise_abort(msg.data)
        raise CanopenError("SDO WRITE U8 failed")

    def sdo_write_u16(self, index, subindex, value):
        data = bytes([
            0x2B,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            value & 0xFF,
            (value >> 8) & 0xFF,
            0, 0
        ])
        with self.lock:
            self.send_frame(COB_SDO_TX, data)
            msg = self.recv_frame(COB_SDO_RX)
        if msg.data[0] == 0x60:
            return
        if msg.data[0] == 0x80:
            self._raise_abort(msg.data)
        raise CanopenError("SDO WRITE U16 failed")

    def sdo_write_u32(self, index, subindex, value):
        data = bytes([
            0x23,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF
        ])
        with self.lock:
            self.send_frame(COB_SDO_TX, data)
            msg = self.recv_frame(COB_SDO_RX)
        if msg.data[0] == 0x60:
            return
        if msg.data[0] == 0x80:
            self._raise_abort(msg.data)
        raise CanopenError("SDO WRITE U32 failed")

    def sdo_write_i32(self, index, subindex, value):
        self.sdo_write_u32(index, subindex, value & 0xFFFFFFFF)

    def sdo_read_u16(self, index, subindex):
        data = bytes([
            0x40,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            0, 0, 0, 0
        ])
        with self.lock:
            self.send_frame(COB_SDO_TX, data)
            msg = self.recv_frame(COB_SDO_RX)
        if msg.data[0] == 0x4B:
            return msg.data[4] | (msg.data[5] << 8)
        if msg.data[0] == 0x80:
            self._raise_abort(msg.data)
        raise CanopenError("SDO READ U16 failed")

    def sdo_read_i32(self, index, subindex):
        data = bytes([
            0x40,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            0, 0, 0, 0
        ])
        with self.lock:
            self.send_frame(COB_SDO_TX, data)
            msg = self.recv_frame(COB_SDO_RX)
        if msg.data[0] == 0x43:
            raw = (
                msg.data[4]
                | (msg.data[5] << 8)
                | (msg.data[6] << 16)
                | (msg.data[7] << 24)
            )
            if raw & 0x80000000:
                raw -= 0x100000000
            return raw
        if msg.data[0] == 0x80:
            self._raise_abort(msg.data)
        raise CanopenError("SDO READ I32 failed")

    def _raise_abort(self, data):
        code = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)
        raise CanopenError(f"SDO_ABORT 0x{code:08X}")

    # =========================
    # CiA402 / EPOS
    # =========================
    def send_nmt_start(self):
        self.send_frame(COB_NMT, bytes([0x01, self.node_id]))

    def write_controlword(self, cw):
        self.sdo_write_u16(0x6040, 0x00, cw)

    def read_statusword(self):
        return self.sdo_read_u16(0x6041, 0x00)

    def read_actual_velocity(self):
        return self.sdo_read_i32(0x606C, 0x00)

    def read_actual_position(self):
        return self.sdo_read_i32(0x6064, 0x00)

    def is_target_reached(self, sw):
        return (sw & (1 << 10)) != 0

    def is_setpoint_ack(self, sw):
        return (sw & (1 << 12)) != 0

    def fault_reset(self):
        self.write_controlword(0x0080)

    def shutdown_drive(self):
        self.write_controlword(0x0006)

    def switch_on_drive(self):
        self.write_controlword(0x0007)

    def enable_operation(self):
        self.write_controlword(0x000F)

    def disable_operation(self):
        self.write_controlword(0x0007)

    def disable_voltage(self):
        self.write_controlword(0x0000)

    def enable_drive(self):
        with self.lock:
            self.fault_reset()
            time.sleep(0.1)
            self.shutdown_drive()
            time.sleep(0.1)
            self.switch_on_drive()
            time.sleep(0.1)
            self.enable_operation()
            time.sleep(0.1)
            self.cfg.enabled = True

    def disable_drive(self):
        with self.lock:
            self.disable_voltage()
            self.cfg.enabled = False

    # =========================
    # Config
    # =========================
    def apply_velocity_mode(self):
        with self.lock:
            self.sdo_write_u8(0x6060, 0x00, 3)
            self.sdo_write_u32(0x607F, 0x00, self.cfg.maxrpm)
            self.sdo_write_u32(0x6083, 0x00, self.cfg.acc)
            self.sdo_write_u32(0x6084, 0x00, self.cfg.dec)
            self.sdo_write_u16(0x6086, 0x00, 0)

    def apply_position_mode(self):
        with self.lock:
            self.sdo_write_u8(0x6060, 0x00, 1)
            self.sdo_write_u32(0x607F, 0x00, self.cfg.maxrpm)
            self.sdo_write_u32(0x6081, 0x00, self.cfg.rpm)
            self.sdo_write_u32(0x6083, 0x00, self.cfg.acc)
            self.sdo_write_u32(0x6084, 0x00, self.cfg.dec)
            self.sdo_write_u16(0x6086, 0x00, 0)
            self.sdo_write_u32(0x6067, 0x00, 50)
            self.sdo_write_u16(0x6068, 0x00, 100)

    def apply(self):
        if self.cfg.mode == "VEL":
            self.apply_velocity_mode()
        else:
            self.apply_position_mode()

    # =========================
    # Run / Stop
    # =========================
    def run_velocity(self):
        with self.lock:
            self.stop_event.clear()
            self.run_active = True
            self.sdo_write_i32(0x60FF, 0x00, self.cfg.rpm)
            self.write_controlword(0x010F)
            time.sleep(0.05)
            self.write_controlword(0x000F)

    def run_position(self):
        self.stop_event.clear()
        self.run_active = True

        with self.lock:
            self.sdo_write_i32(0x607A, 0x00, self.cfg.pos)

            if self.cfg.posmode == "ABS":
                cw_prepare = 0x002F
                cw_start = 0x003F
            else:
                cw_prepare = 0x006F
                cw_start = 0x007F

            self.write_controlword(cw_prepare)
            time.sleep(0.02)
            self.write_controlword(cw_start)
            time.sleep(0.02)
            self.write_controlword(cw_prepare)

        while not self.stop_event.is_set():
            sw = self.read_statusword()
            if self.is_target_reached(sw):
                print("OK TARGET_REACHED")
                self.run_active = False
                return
            time.sleep(0.1)

        self.run_active = False

    def run(self):
        if self.cfg.mode == "VEL":
            self.run_velocity()
        else:
            t = threading.Thread(target=self.run_position, daemon=True)
            t.start()

    def stop(self):
        self.stop_event.set()
        with self.lock:
            if self.cfg.mode == "VEL":
                self.sdo_write_i32(0x60FF, 0x00, 0)
                self.write_controlword(0x010F)
                time.sleep(0.03)
                self.write_controlword(0x000F)
                time.sleep(0.03)
                self.sdo_write_i32(0x60FF, 0x00, 0)
                self.write_controlword(0x010F)
                time.sleep(0.03)
                self.write_controlword(0x000F)
            else:
                self.disable_operation()
                time.sleep(0.03)
                self.disable_voltage()
                self.cfg.enabled = False
        self.run_active = False
        print("OK STOP")

    # =========================
    # Telemetry
    # =========================
    def telem_once(self):
        try:
            vel = self.read_actual_velocity()
        except Exception:
            vel = "ERR"

        try:
            pos = self.read_actual_position()
        except Exception:
            pos = "ERR"

        print(f"TELEM VEL={vel} POS={pos}")

    def status(self):
        try:
            sw = self.read_statusword()
        except Exception:
            sw = "ERR"

        try:
            vel = self.read_actual_velocity()
        except Exception:
            vel = "ERR"

        try:
            pos = self.read_actual_position()
        except Exception:
            pos = "ERR"

        tr = self.is_target_reached(sw) if isinstance(sw, int) else "ERR"
        ack = self.is_setpoint_ack(sw) if isinstance(sw, int) else "ERR"

        print(
            f"STAT MODE={self.cfg.mode} POSMODE={self.cfg.posmode} "
            f"RPM={self.cfg.rpm} MAXRPM={self.cfg.maxrpm} "
            f"ACC={self.cfg.acc} DEC={self.cfg.dec} POS_CMD={self.cfg.pos} "
            f"EN={1 if self.cfg.enabled else 0} SW={sw} VEL={vel} POS_ACT={pos} "
            f"TR={tr} ACK={ack}"
        )

    def telem_loop(self):
        while self.telem_enabled:
            self.telem_once()
            time.sleep(0.2)

    def telem_on(self):
        if self.telem_enabled:
            return
        self.telem_enabled = True
        self.telem_thread = threading.Thread(target=self.telem_loop, daemon=True)
        self.telem_thread.start()
        print("OK TELEM_ON")

    def telem_off(self):
        self.telem_enabled = False
        print("OK TELEM_OFF")


def print_help():
    print("""
Komendy:
  help
  nmt
  enable
  disable
  mode vel
  mode pos
  posmode abs
  posmode rel
  set rpm <val>
  set maxrpm <val>
  set acc <val>
  set dec <val>
  set pos <val>
  apply
  run
  stop
  telem
  telem on
  telem off
  status
  quit
""")


def main():
    ctrl = EposController(channel=CAN_CHANNEL, node_id=NODE_ID)
    print("EPOS4 Raspberry Pi CAN CLI")
    print_help()

    try:
        ctrl.send_nmt_start()
        print("OK NMT_START")
    except Exception as e:
        print(f"ERR NMT_START {e}")

    while True:
        try:
            line = input("epos> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == "help":
                print_help()

            elif cmd == "quit":
                break

            elif cmd == "nmt":
                ctrl.send_nmt_start()
                print("OK NMT_START")

            elif cmd == "enable":
                ctrl.enable_drive()
                print("OK ENABLE")

            elif cmd == "disable":
                ctrl.disable_drive()
                print("OK DISABLE")

            elif cmd == "mode" and len(parts) == 2:
                v = parts[1].upper()
                if v in ("VEL", "POS"):
                    ctrl.cfg.mode = v
                    print(f"OK MODE {v}")
                else:
                    print("ERR MODE")

            elif cmd == "posmode" and len(parts) == 2:
                v = parts[1].upper()
                if v in ("ABS", "REL"):
                    ctrl.cfg.posmode = v
                    print(f"OK POSMODE {v}")
                else:
                    print("ERR POSMODE")

            elif cmd == "set" and len(parts) == 3:
                key = parts[1].lower()
                val = int(parts[2])

                if key == "rpm":
                    ctrl.cfg.rpm = val
                    print("OK SET RPM")
                elif key == "maxrpm":
                    ctrl.cfg.maxrpm = val
                    print("OK SET MAXRPM")
                elif key == "acc":
                    ctrl.cfg.acc = val
                    print("OK SET ACC")
                elif key == "dec":
                    ctrl.cfg.dec = val
                    print("OK SET DEC")
                elif key == "pos":
                    ctrl.cfg.pos = val
                    print("OK SET POS")
                else:
                    print("ERR SET KEY")

            elif cmd == "apply":
                ctrl.apply()
                print("OK APPLY")

            elif cmd == "run":
                ctrl.run()
                print("OK RUN")

            elif cmd == "stop":
                ctrl.stop()

            elif cmd == "telem" and len(parts) == 1:
                ctrl.telem_once()

            elif cmd == "telem" and len(parts) == 2 and parts[1].lower() == "on":
                ctrl.telem_on()

            elif cmd == "telem" and len(parts) == 2 and parts[1].lower() == "off":
                ctrl.telem_off()

            elif cmd == "status":
                ctrl.status()

            else:
                print("ERR UNKNOWN")

        except Exception as e:
            print(f"ERR {e}")

    ctrl.telem_off()
    try:
        ctrl.bus.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()