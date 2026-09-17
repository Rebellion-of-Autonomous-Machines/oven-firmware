# RS-485 / Modbus (дверь)

Параметры slave:
- UART: `Serial2`
- RXD: `GPIO14`
- TXD: `GPIO27`
- Скорость: `9600 8N1`
- Slave ID: `1`

## Holding registers

- `0` `RW` `COMMAND`  
  После обработки команда сбрасывается прошивкой в `0`.
- `1` `RO` `DIRECTION`  
  `0` = stop, `1` = open, `2` = close
- `2` `RO` `STATUS_BITS`  
  bit0 = door moving
- `3` `RO` `COMMAND_RESULT`  
  `0` = ok, `1` = unknown command

## Команды (`reg 0`)

- `1` открыть дверь
- `2` закрыть дверь
- `3` стоп двери

## Тестер

Файл: `tools/rs485_door_tester.py`

Установка зависимостей:
```bash
pip install pymodbus pyserial
```

Запуск:
```bash
python tools/rs485_door_tester.py
```
