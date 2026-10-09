import inspect
import tkinter as tk
from tkinter import messagebox, ttk

from pymodbus.client import ModbusTcpClient

HREG_STAT_MODE = 0
HREG_STAT_ERROR = 1
HREG_MAN_CONTROL_C = 2
HREG_MAN_CONTROL_X = 3
HREG_MAN_CONTROL_Y = 4
HREG_MAN_CONTROL_Z = 5
HREG_COMMAND = 6
HREG_COMMAND_RESULT = 7
HREG_DIRECTION = 8
HREG_STATUS_BITS = 9
HREG_DI_BITS = 10

IREG_TIME_ALL = 0
IREG_TIME_HEAT = 1
IREG_END_OPEN = 20
IREG_END_CLOSED = 21

STAT_MANUAL = 20
STAT_EMERGENCY_STOP = 30

MAN_CMD_OPEN = 1
MAN_CMD_CLOSE = 2

STATE_TEXT = {
    1: "INITIAL",
    11: "SCENARIO_ACTIVE",
    19: "SCENARIO_COMPLETE",
    20: "MANUAL",
    30: "EMERGENCY_STOP",
    99: "ERROR",
}

ERROR_TEXT = {
    0: "NONE",
    100: "MANUAL_MODE_REQUIRED",
    101: "COMMAND_DURING_MOVEMENT",
    102: "INVALID_COMMAND",
    103: "REMOTE_CONTROL_LOCKED",
    105: "DIRECT_RELAY_CONTROL_FORBIDDEN",
}

RESULT_TEXT = {
    0: "OK",
    1: "UNKNOWN_CMD",
    2: "BUSY",
    3: "MODE_REQUIRED",
    4: "INVALID_ARG",
    5: "REMOTE_LOCKED",
    6: "RELAY_CONTROL_FORBIDDEN",
}

DIRECTION_TEXT = {0: "Стоп", 1: "Открытие", 2: "Закрытие"}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Modbus TCP: дверь печи")
        self.root.geometry("760x520")
        self.root.minsize(700, 480)
        self.client = None
        self.connected = False

        self.host_var = tk.StringVar(value="192.168.1.52")
        self.port_var = tk.StringVar(value="502")
        self.slave_var = tk.StringVar(value="1")
        self.status_var = tk.StringVar(value="Не подключено")
        self.values_var = tk.StringVar(value="-")
        self.control_widgets = []

        self._build_ui()
        self._tick()

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        connection = ttk.LabelFrame(frame, text="Подключение", padding=8)
        connection.grid(row=0, column=0, sticky="ew")
        ttk.Label(connection, text="IP:").grid(row=0, column=0)
        ttk.Entry(connection, textvariable=self.host_var, width=16).grid(row=0, column=1)
        ttk.Label(connection, text="Порт:").grid(row=0, column=2, padx=(8, 0))
        ttk.Entry(connection, textvariable=self.port_var, width=9).grid(row=0, column=3)
        ttk.Label(connection, text="Slave ID:").grid(row=0, column=4, padx=(8, 0))
        ttk.Entry(connection, textvariable=self.slave_var, width=6).grid(row=0, column=5)
        ttk.Button(connection, text="Подключить", command=self.connect).grid(row=0, column=6, padx=(8, 0))
        ttk.Button(connection, text="Отключить", command=self.disconnect).grid(row=0, column=7, padx=(4, 0))

        controls = ttk.LabelFrame(frame, text="Man_Control (40003-40006)", padding=8)
        controls.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        open_button = ttk.Button(controls, text="Открыть", command=lambda: self.send_manual_command(MAN_CMD_OPEN))
        close_button = ttk.Button(controls, text="Закрыть", command=lambda: self.send_manual_command(MAN_CMD_CLOSE))
        open_button.grid(row=0, column=0, padx=4, pady=4)
        close_button.grid(row=0, column=1, padx=4, pady=4)
        self.control_widgets.extend([open_button, close_button])

        service = ttk.LabelFrame(frame, text="Состояние автоматики", padding=8)
        service.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(
            service,
            text="Сброс ошибки / режим 20",
            command=lambda: self.write_reg(HREG_STAT_MODE, STAT_MANUAL),
        ).grid(row=0, column=0, padx=4)
        ttk.Button(
            service,
            text="Аварийная остановка (30)",
            command=lambda: self.write_reg(HREG_STAT_MODE, STAT_EMERGENCY_STOP),
        ).grid(row=0, column=1, padx=4)

        status = ttk.LabelFrame(frame, text="Регистры", padding=8)
        status.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(
            status,
            textvariable=self.values_var,
            justify="left",
            anchor="w",
            wraplength=710,
        ).grid(row=1, column=0, sticky="w")

    def connect(self):
        self.disconnect()
        try:
            host = self.host_var.get().strip()
            port = int(self.port_var.get())
            unit = self._slave()
            if not host or not 1 <= port <= 65535 or not 0 <= unit <= 255:
                raise ValueError("Проверьте IP, порт (1...65535) и Unit ID (0...255)")
            self.client = ModbusTcpClient(host=host, port=port, timeout=1)
            self.connected = bool(self.client.connect())
            self.status_var.set("Подключено" if self.connected else "Ошибка подключения")
        except Exception as exc:
            self.connected = False
            self.status_var.set(f"Ошибка: {exc}")

    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except Exception:
                pass
        self.client = None
        self.connected = False

    def _slave(self):
        return int(self.slave_var.get())

    def _call_with_slave(self, fn, *args, **kwargs):
        params = inspect.signature(fn).parameters
        if "slave" in params:
            kwargs["slave"] = self._slave()
        elif "unit" in params:
            kwargs["unit"] = self._slave()
        elif "device_id" in params:
            kwargs["device_id"] = self._slave()
        return fn(*args, **kwargs)

    def write_reg(self, address: int, value: int):
        try:
            if not self.connected or not self.client:
                raise RuntimeError("Нет подключения")
            response = self._call_with_slave(self.client.write_register, address, value & 0xFFFF)
            if response.isError():
                raise RuntimeError(str(response))
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def write_regs(self, address: int, values):
        if not self.connected or not self.client:
            raise RuntimeError("Нет подключения")
        response = self._call_with_slave(self.client.write_registers, address, values)
        if response.isError():
            raise RuntimeError(str(response))

    def send_manual_command(self, command: int, x: int = 0, y: int = 0, z: int = 0):
        try:
            # One transaction sets Stat_Mode=20 and all CXYZ registers.
            self.write_regs(HREG_STAT_MODE, [STAT_MANUAL, 0, command, x, y, z])
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def read_holding(self, address: int, count: int):
        fn = self.client.read_holding_registers
        kwargs = {"count": count} if "count" in inspect.signature(fn).parameters else {}
        response = self._call_with_slave(fn, address, **kwargs)
        if response.isError():
            raise RuntimeError(str(response))
        return response.registers

    def read_input(self, address: int, count: int):
        fn = self.client.read_input_registers
        kwargs = {"count": count} if "count" in inspect.signature(fn).parameters else {}
        response = self._call_with_slave(fn, address, **kwargs)
        if response.isError():
            raise RuntimeError(str(response))
        return response.registers

    def poll(self):
        if not self.connected or not self.client:
            return

        holding = self.read_holding(0, 11)
        time_regs = self.read_input(0, 2)
        ends = self.read_input(20, 2)

        state = holding[HREG_STAT_MODE]
        error = holding[HREG_STAT_ERROR]
        result = holding[HREG_COMMAND_RESULT]
        direction = holding[HREG_DIRECTION]
        status_bits = holding[HREG_STATUS_BITS]
        di_bits = holding[HREG_DI_BITS]
        remote_enabled = bool(status_bits & (1 << 3))

        for widget in self.control_widgets:
            widget.state(["!disabled"] if remote_enabled else ["disabled"])

        lines = [
            f"40001 Stat_Mode={state} ({STATE_TEXT.get(state, 'UNKNOWN')})",
            f"40002 Stat_Error={error} ({ERROR_TEXT.get(error, 'UNKNOWN')})",
            f"40008 Command_Result={result} ({RESULT_TEXT.get(result, 'UNKNOWN')})",
            f"40009 Direction={direction} ({DIRECTION_TEXT.get(direction, 'UNKNOWN')})",
            f"40010 StatusBits=0x{status_bits:04X}  40011 DI5={di_bits & 1} DI6={(di_bits >> 1) & 1}",
            f"30001 TimeAll={time_regs[0]} h  30002 TimeHeat={time_regs[1]} h",
            f"30021 EndOpen={ends[0]}  30022 EndClosed={ends[1]}",
        ]
        self.values_var.set("\n".join(lines))

    def _tick(self):
        try:
            self.poll()
        except Exception as exc:
            self.status_var.set(f"Ошибка: {exc}")
        self.root.after(1000, self._tick)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
