import sys
from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QLineEdit, QMessageBox,
    QGroupBox, QRadioButton
)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EPOS4 GUI")
        self.resize(980, 620)

        self.serial = QSerialPort(self)
        self.serial.setBaudRate(115200)
        self.serial.readyRead.connect(self.on_ready_read)
        self.rx_buffer = ""

        self.telem_timer = QTimer(self)
        self.telem_timer.timeout.connect(self.request_telem)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.request_status)

        self.last_status = {}
        self.last_telem = {}

        self.telem_pending = False
        self.status_pending = False

        self.build_ui()
        self.refresh_ports()
        self.update_mode_controls()
        self.update_buttons_from_state()

    def build_ui(self):
        root = QVBoxLayout(self)

        port_box = QGroupBox("Połączenie")
        port_layout = QHBoxLayout(port_box)

        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Odśwież")
        self.connect_btn = QPushButton("Połącz")
        self.disconnect_btn = QPushButton("Rozłącz")

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_port)
        self.disconnect_btn.clicked.connect(self.disconnect_port)

        port_layout.addWidget(QLabel("Port:"))
        port_layout.addWidget(self.port_combo, 1)
        port_layout.addWidget(self.refresh_btn)
        port_layout.addWidget(self.connect_btn)
        port_layout.addWidget(self.disconnect_btn)

        mode_box = QGroupBox("Tryb")
        mode_layout = QHBoxLayout(mode_box)

        self.vel_radio = QRadioButton("Velocity")
        self.pos_radio = QRadioButton("Position")
        self.vel_radio.setChecked(True)

        self.vel_radio.toggled.connect(self.update_mode_controls)
        self.pos_radio.toggled.connect(self.update_mode_controls)

        mode_layout.addWidget(self.vel_radio)
        mode_layout.addWidget(self.pos_radio)
        mode_layout.addStretch()

        posmode_box = QGroupBox("Tryb pozycji")
        posmode_layout = QHBoxLayout(posmode_box)

        self.abs_radio = QRadioButton("Absolute")
        self.rel_radio = QRadioButton("Relative")
        self.abs_radio.setChecked(True)

        posmode_layout.addWidget(self.abs_radio)
        posmode_layout.addWidget(self.rel_radio)
        posmode_layout.addStretch()

        params_box = QGroupBox("Parametry")
        params_layout = QGridLayout(params_box)

        self.rpm_edit = QLineEdit("200")
        self.maxrpm_edit = QLineEdit("7000")
        self.acc_edit = QLineEdit("10")
        self.dec_edit = QLineEdit("10")
        self.pos_edit = QLineEdit("0")

        params_layout.addWidget(QLabel("RPM:"), 0, 0)
        params_layout.addWidget(self.rpm_edit, 0, 1)

        params_layout.addWidget(QLabel("MAX RPM:"), 1, 0)
        params_layout.addWidget(self.maxrpm_edit, 1, 1)

        params_layout.addWidget(QLabel("Acceleration:"), 2, 0)
        params_layout.addWidget(self.acc_edit, 2, 1)

        params_layout.addWidget(QLabel("Deceleration:"), 3, 0)
        params_layout.addWidget(self.dec_edit, 3, 1)

        params_layout.addWidget(QLabel("Position:"), 4, 0)
        params_layout.addWidget(self.pos_edit, 4, 1)

        control_box = QGroupBox("Sterowanie")
        control_layout = QGridLayout(control_box)

        self.apply_btn = QPushButton("Apply")
        self.enable_btn = QPushButton("Enable")
        self.disable_btn = QPushButton("Disable")
        self.run_btn = QPushButton("Run")
        self.stop_btn = QPushButton("Stop")
        self.status_btn = QPushButton("Status")

        self.apply_btn.clicked.connect(self.apply_config)
        self.enable_btn.clicked.connect(lambda: self.send_command("ENABLE"))
        self.disable_btn.clicked.connect(lambda: self.send_command("DISABLE"))
        self.run_btn.clicked.connect(lambda: self.send_command("RUN"))
        self.stop_btn.clicked.connect(lambda: self.send_command("STOP"))
        self.status_btn.clicked.connect(self.request_status)

        control_layout.addWidget(self.apply_btn, 0, 0)
        control_layout.addWidget(self.enable_btn, 0, 1)
        control_layout.addWidget(self.disable_btn, 0, 2)
        control_layout.addWidget(self.run_btn, 1, 0)
        control_layout.addWidget(self.stop_btn, 1, 1)
        control_layout.addWidget(self.status_btn, 1, 2)

        self.telem_label = QLabel("Brak telemetrii")
        self.telem_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.telem_label.setWordWrap(True)

        self.status_label = QLabel("Brak statusu")
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.status_label.setWordWrap(True)

        self.info_label = QLabel("")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.info_label.setWordWrap(True)

        root.addWidget(port_box)
        root.addWidget(mode_box)
        root.addWidget(posmode_box)
        root.addWidget(params_box)
        root.addWidget(control_box)
        root.addWidget(QLabel("Telemetria szybka:"))
        root.addWidget(self.telem_label)
        root.addWidget(QLabel("Status pełny:"))
        root.addWidget(self.status_label)
        root.addWidget(QLabel("Komunikaty:"))
        root.addWidget(self.info_label)

    def update_mode_controls(self):
        connected = self.serial.isOpen()
        is_pos = self.pos_radio.isChecked()
        self.abs_radio.setEnabled(connected and is_pos)
        self.rel_radio.setEnabled(connected and is_pos)
        self.pos_edit.setEnabled(connected and is_pos)

    @Slot()
    def refresh_ports(self):
        self.port_combo.clear()
        ports = QSerialPortInfo.availablePorts()
        for port in ports:
            label = f"{port.portName()} | {port.description()}"
            self.port_combo.addItem(label, port.portName())

    @Slot()
    def connect_port(self):
        if self.serial.isOpen():
            return

        if self.port_combo.currentIndex() < 0:
            QMessageBox.warning(self, "Błąd", "Brak portu COM.")
            return

        port_name = self.port_combo.currentData()
        self.serial.setPortName(port_name)

        ok = self.serial.open(QSerialPort.ReadWrite)
        if not ok:
            err = self.serial.errorString()
            QMessageBox.critical(
                self,
                "Błąd",
                f"Nie udało się otworzyć portu {port_name}.\n\nSzczegóły: {err}"
            )
            return

        self.last_status = {}
        self.last_telem = {}
        self.telem_pending = False
        self.status_pending = False

        self.telem_label.setText("Połączono. Oczekiwanie na telemetrię...")
        self.status_label.setText("Połączono. Oczekiwanie na status...")
        self.info_label.setText("")

        self.update_buttons_from_state()
        self.update_mode_controls()

        self.telem_timer.start(100)
        self.status_timer.start(1000)

        self.request_telem()
        self.request_status()

    @Slot()
    def disconnect_port(self):
        self.telem_timer.stop()
        self.status_timer.stop()

        if self.serial.isOpen():
            self.serial.close()

        self.last_status = {}
        self.last_telem = {}
        self.telem_pending = False
        self.status_pending = False

        self.telem_label.setText("Brak telemetrii")
        self.status_label.setText("Brak statusu")
        self.info_label.setText("")

        self.update_buttons_from_state()
        self.update_mode_controls()

    def send_command(self, cmd: str) -> bool:
        if not self.serial.isOpen():
            QMessageBox.warning(self, "Błąd", "Najpierw połącz port.")
            return False

        payload = (cmd.strip() + "\n").encode("utf-8")
        written = self.serial.write(payload)
        return written != -1

    def apply_config(self):
        mode = "VEL" if self.vel_radio.isChecked() else "POS"
        posmode = "ABS" if self.abs_radio.isChecked() else "REL"

        rpm = self.rpm_edit.text().strip()
        maxrpm = self.maxrpm_edit.text().strip()
        acc = self.acc_edit.text().strip()
        dec = self.dec_edit.text().strip()
        pos = self.pos_edit.text().strip()

        if not rpm or not maxrpm or not acc or not dec:
            QMessageBox.warning(self, "Błąd", "RPM, MAX RPM, ACC i DEC muszą być ustawione.")
            return

        commands = [
            f"MODE {mode}",
            f"SET RPM {rpm}",
            f"SET MAXRPM {maxrpm}",
            f"SET ACC {acc}",
            f"SET DEC {dec}",
        ]

        if mode == "POS":
            if not pos:
                QMessageBox.warning(self, "Błąd", "Pozycja musi być ustawiona.")
                return
            commands.append(f"POSMODE {posmode}")
            commands.append(f"SET POS {pos}")

        commands.append("APPLY")

        for cmd in commands:
            if not self.send_command(cmd):
                QMessageBox.warning(self, "Błąd", f"Nie udało się wysłać komendy: {cmd}")
                return

    def is_drive_enabled(self):
        return self.last_status.get("EN", "0") in ("1", "true", "TRUE", "ON")

    def update_buttons_from_state(self):
        connected = self.serial.isOpen()
        enabled = self.is_drive_enabled() if connected else False

        self.refresh_btn.setEnabled(not connected)
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.port_combo.setEnabled(not connected)

        self.vel_radio.setEnabled(connected)
        self.pos_radio.setEnabled(connected)

        self.rpm_edit.setEnabled(connected)
        self.maxrpm_edit.setEnabled(connected)
        self.acc_edit.setEnabled(connected)
        self.dec_edit.setEnabled(connected)

        is_pos = self.pos_radio.isChecked()
        self.abs_radio.setEnabled(connected and is_pos)
        self.rel_radio.setEnabled(connected and is_pos)
        self.pos_edit.setEnabled(connected and is_pos)

        self.apply_btn.setEnabled(connected)
        self.run_btn.setEnabled(connected)
        self.stop_btn.setEnabled(connected)

        self.enable_btn.setEnabled(connected and not enabled)
        self.disable_btn.setEnabled(connected and enabled)

        self.status_btn.setEnabled(connected)

    @Slot()
    def request_telem(self):
        if not self.serial.isOpen():
            return
        if self.telem_pending:
            return
        if self.send_command("TELEM"):
            self.telem_pending = True

    @Slot()
    def request_status(self):
        if not self.serial.isOpen():
            return
        if self.status_pending:
            return
        if self.send_command("STATUS"):
            self.status_pending = True

    @Slot()
    def on_ready_read(self):
        data = self.serial.readAll().data().decode("utf-8", errors="ignore")
        self.rx_buffer += data

        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            if line.startswith("TELEM "):
                self.telem_pending = False

                parts = {}
                for token in line.split()[1:]:
                    if "=" in token:
                        k, v = token.split("=", 1)
                        parts[k] = v

                self.last_telem = parts

                vel = parts.get("VEL", "?")
                pos = parts.get("POS", "?")
                self.telem_label.setText(f"Vel actual: {vel} | Pos actual: {pos}")
                continue

            if line.startswith("STAT "):
                self.status_pending = False

                parts = {}
                for token in line.split()[1:]:
                    if "=" in token:
                        k, v = token.split("=", 1)
                        parts[k] = v

                self.last_status = parts

                mode = parts.get("MODE", "?")
                posmode = parts.get("POSMODE", "?")
                rpm = parts.get("RPM", "?")
                maxrpm = parts.get("MAXRPM", "?")
                vel = parts.get("VEL", "?")
                pos_cmd = parts.get("POS_CMD", "?")
                pos_act = parts.get("POS_ACT", "?")
                tr = parts.get("TR", "?")
                ack = parts.get("ACK", "?")
                sw = parts.get("SW", "?")
                en = parts.get("EN", "?")
                acc = parts.get("ACC", "?")
                dec = parts.get("DEC", "?")

                pretty = (
                    f"Tryb: {mode} | PosMode: {posmode} | EN: {en} | "
                    f"RPM: {rpm} | MAX RPM: {maxrpm} | ACC: {acc} | DEC: {dec} | "
                    f"Vel actual: {vel} | Pos cmd: {pos_cmd} | Pos actual: {pos_act} | "
                    f"Target reached: {tr} | Ack: {ack} | SW: {sw}"
                )
                self.status_label.setText(pretty)
                self.update_buttons_from_state()
                self.update_mode_controls()
                continue

            if line.startswith("OK ") or line.startswith("ERR "):
                self.info_label.setText(line)
                continue


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())