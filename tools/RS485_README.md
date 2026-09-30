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
  bit1 = door open  
  bit2 = door closed  
  bit3 = web/Modbus commands enabled  
  bit4 = manual open selected (`DI5=1`, `DI6=1`)  
  bit5 = manual close selected (`DI5=0`, `DI6=1`)  
  bit6 = invalid input combination or input read error
- `3` `RO` `COMMAND_RESULT`  
  `0` = ok, `1` = unknown command, `2` = busy

## Команды (`reg 0`)

- `1` открыть дверь
- `2` закрыть дверь

Во время 6-секундного хода повторные и противоположные команды отклоняются с результатом `busy`. После завершения хода реле остаётся в выбранном положении, и становится доступна только команда движения в противоположную сторону.

## Режимы DI5/DI6

- `1/0` — автоматический режим, веб и Modbus разрешены.
- `1/1` — ручное открытие, веб и Modbus заблокированы.
- `0/1` — ручное закрытие, веб и Modbus заблокированы.
- `0/0` — недопустимое состояние, веб и Modbus заблокированы.

Ручная команда не прерывает текущий ход. После его завершения выполняется последняя полученная ручная команда.

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
