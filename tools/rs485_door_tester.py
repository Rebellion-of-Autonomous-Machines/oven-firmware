import inspect
import tkinter as tk
from tkinter import ttk, messagebox

from pymodbus.client import ModbusSerialClient

REG_COMMAND = 0
REG_DIRECTION = 1
REG_STATUS_BITS = 2
REG_COMMAND_RESULT = 3

CMD_NONE = 0
CMD_DOOR_OPEN = 1
CMD_DOOR_CLOSE = 2
CMD_DOOR_STOP = 3

CMD_RESULT_TEXT = {
    0: "OK",
    1: "UNKNOWN_CMD",
}

DIRECTION_TEXT = {
    0: "Стоп",
    1: "Открытие",
    2: "Закрытие",
}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RS-485 Modbus тест двери")
        self.root.geometry("560x340")
        self.root.minsize(520, 320)

        self.client = None
        self.connected = False

        self.port_var = tk.StringVar(value="COM4")
        self.baud_var = tk.StringVar(value="9600")
        self.slave_var = tk.StringVar(value="1")

        self.status_var = tk.StringVar(value="Не подключено")
        self.values_var = tk.StringVar(value="-")

        self._build_ui()
        self._tick()

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)

        conn = ttk.LabelFrame(frm, text="Подключение", padding=8)
        conn.grid(row=0, column=0, sticky="ew")

        ttk.Label(conn, text="COM:").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.port_var, width=12).grid(row=0, column=1, sticky="w")
        ttk.Label(conn, text="Baud:").grid(row=0, column=2, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=3, sticky="w")
        ttk.Label(conn, text="Slave ID:").grid(row=0, column=4, sticky="w", padx=(8, 0))
        ttk.Entry(conn, textvariable=self.slave_var, width=8).grid(row=0, column=5, sticky="w")

        ttk.Button(conn, text="Подключить", command=self.connect).grid(row=0, column=6, padx=(8, 0))
        ttk.Button(conn, text="Отключить", command=self.disconnect).grid(row=0, column=7, padx=(4, 0))

        door = ttk.LabelFrame(frm, text="Дверь", padding=8)
        door.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(door, text="Открыть", command=lambda: self.send_command(CMD_DOOR_OPEN)).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(door, text="Закрыть", command=lambda: self.send_command(CMD_DOOR_CLOSE)).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(door, text="Стоп", command=lambda: self.send_command(CMD_DOOR_STOP)).grid(row=0, column=2, padx=4, pady=4)

        stat = ttk.LabelFrame(frm, text="Статус", padding=8)
        stat.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(stat, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(stat, textvariable=self.values_var, justify="left", anchor="w", wraplength=520).grid(row=1, column=0, sticky="w")

    def connect(self):
        self.disconnect()
        try:
            baud = int(self.baud_var.get())
            self.client = ModbusSerialClient(
                port=self.port_var.get().strip(),
                baudrate=baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=1,
            )
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
        sid = self._slave()
        params = inspect.signature(fn).parameters
        if "slave" in params:
            kwargs["slave"] = sid
        elif "unit" in params:
            kwargs["unit"] = sid
        elif "device_id" in params:
            kwargs["device_id"] = sid
        return fn(*args, **kwargs)

    def write_reg(self, addr: int, value: int):
        if not self.connected or not self.client:
            raise RuntimeError("Нет подключения")
        rr = self._call_with_slave(self.client.write_register, addr, int(value) & 0xFFFF)
        if rr.isError():
            raise RuntimeError(str(rr))

    def send_command(self, cmd: int):
        try:
            self.write_reg(REG_COMMAND, cmd)
        except Exception as exc:
            messagebox.showerror("Modbus", str(exc))

    def poll(self):
        if not self.connected or not self.client:
            return

        read_fn = self.client.read_holding_registers
        params = inspect.signature(read_fn).parameters
        kwargs = {}
        if "count" in params:
            kwargs["count"] = 4
        rr = self._call_with_slave(read_fn, 0, **kwargs)

        if rr.isError():
            self.status_var.set(f"Ошибка чтения: {rr}")
            return

        r = rr.registers
        direction = r[REG_DIRECTION]
        status_bits = r[REG_STATUS_BITS]
        cmd_res = r[REG_COMMAND_RESULT]

        direction_text = DIRECTION_TEXT.get(direction, f"UNKNOWN({direction})")
        cmd_res_text = CMD_RESULT_TEXT.get(cmd_res, f"UNKNOWN({cmd_res})")
        moving = "Да" if (status_bits & (1 << 0)) else "Нет"

        lines = [
            f"Direction={direction} ({direction_text})",
            f"Moving={moving}  StatusBits=0x{status_bits:04X}",
            f"CommandResult={cmd_res} ({cmd_res_text})",
        ]
        self.values_var.set("\n".join(lines))

    def _tick(self):
        try:
            self.poll()
        except Exception as exc:
            self.status_var.set(f"Ошибка: {exc}")
        self.root.after(500, self._tick)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
