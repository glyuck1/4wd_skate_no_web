# Конфигурация ODrive Pro

Четыре контроллера **ODrive Pro**, по одному на колесо. Каждый работает как один узел CAN (`axis0`).
Файлы — это экспорт `odrivetool backup-config` (снят 1 марта 2026).

| Файл | CAN node_id | Колесо | Ось в прошивке |
|---|---|---|---|
| `node0_FL.json` | 0 | Переднее левое  | `AXIS_FL` |
| `node1_FR.json` | 1 | Переднее правое | `AXIS_FR` |
| `node2_RL.json` | 2 | Заднее левое    | `AXIS_RL` |
| `node3_RR.json` | 3 | Заднее правое   | `AXIS_RR` |

## Общие параметры (одинаковы на всех 4)

| Параметр | Значение | Комментарий |
|---|---|---|
| `can.config.baud_rate` | 500 000 | совпадает с `TWAI_TIMING_CONFIG_500KBITS()` в прошивке |
| `can.config.protocol` | 1 (CAN Simple) | стандартные 11-бит ID: `node_id << 5 \| cmd_id` |
| Энкодер | Hall (`hall_encoder0`, `load/commutation_encoder = 8`) | датчики Холла мотор-колеса |
| `dc_bus_undervoltage_trip_level` | 33 V | ODrive сам уходит в ошибку ниже 33 V |
| `dc_bus_overvoltage_trip_level` | 60 V | |
| `max_regen_current` | 0 A | рекуперация в батарею выключена на уровне платы |
| `controller.vel_limit` | 45 rev/s | + `enable_torque_mode_vel_limit = true` — момент режется у лимита |
| `enable_overspeed_error` | true | |
| `enable_watchdog` | true | таймаут разный — см. ниже |
| Циклические CAN-сообщения | 100 мс | heartbeat, encoder, Iq, temp, bus V/I, error, torques |
| Startup-флаги | все `false` | калибровка и closed loop запускаются только командой с ESP32 |

## Различия между узлами — проверить!

| Параметр | FL (0) | FR (1) | RL (2) | RR (3) |
|---|---|---|---|---|
| `motor.pole_pairs` | **7** | **7** | **14** | **14** |
| `motor.torque_constant`, Nm/A | 0.0486 | 0.0486 | 0.0827 | 0.0827 |
| `motor.phase_resistance`, Ω | 0.0417 | 0.0450 | 0.0416 | 0.0443 |
| `motor.phase_inductance`, µH | 26.1 | 26.3 | 25.3 | 26.4 |
| `motor.current_soft_max / hard_max`, A | 90 / 110 | 90 / 110 | 40 / 60 | 40 / 60 |
| `motor.current_control_bandwidth`, rad/s | 1000 | 1000 | **50** | **50** |
| `dc_max_positive / negative_current`, A | 90 / −30 | 90 / −30 | 30 / −17 | 30 / −17 |
| `torque_soft_min/max`, Nm | ±4 | ±4 | ±5 | ±5 |
| `controller.control_mode` | 1 TORQUE | **2 VELOCITY** | 1 TORQUE | 1 TORQUE |
| `controller.input_mode` | 6 TORQUE_RAMP | **2 VEL_RAMP** | 6 TORQUE_RAMP | 6 TORQUE_RAMP |
| `controller.torque_ramp_rate`, Nm/s | 10 | **0.01** | 15 | 15 |
| `watchdog_timeout`, s | 2.0 | 1.0 | 0.2 | 0.2 |

### Что из этого следует

1. **FR (node 1) в режиме VELOCITY_CONTROL.** Прошивка `4wd_skate_no_web` шлёт `Set_Input_Torque` (0x0E),
   а в режиме скорости ODrive его не использует → переднее правое колесо не тянет.
   Для прошивки без веба нужно: `control_mode = 1`, `input_mode = 6` (или 1), `torque_ramp_rate` как у остальных.
   Для веб-версии (`4WD_SKATE`, команда `Set_Input_Vel` 0x0D) наоборот — все четыре в `control_mode = 2`.
   *Если конфиги уже меняли после 1 марта — сверить с живыми платами.*
2. **Полюса: 7 спереди, 14 сзади** при практически одинаковых R и L всех четырёх моторов — похоже, что моторы одинаковые,
   а `pole_pairs` выставлен по-разному. Hall-коммутация от этого не ломается, но:
   - механическая скорость (`vel_estimate`) спереди и сзади отличается в 2 раза → пороги торможения
     (`BRAKE_VEL_THRESH`) и traction control в веб-версии считают неправильно;
   - при одинаковой команде момента ток различается в ~1.7 раза (`Iq = T / Kt`) → тяга смещена вперёд.
   Проверка: в `odrivetool` повернуть колесо рукой ровно на 1 оборот и посмотреть, насколько изменился
   `axis0.pos_estimate` (должно быть ≈ 1.0). Либо посчитать магниты: пар полюсов = магниты / 2.
3. **`current_control_bandwidth = 50` сзади** (на передних и по умолчанию — 1000). Токовый контур сзади очень медленный —
   задние моторы реагируют на момент с задержкой. Если сзади есть раскачка или вялость на скорости, начать с этого.
4. **Watchdog разный.** Прошивка шлёт момент 50 Гц (каждые 20 мс) — 0.2 с достаточно, можно выставить одинаково на всех.
5. `node2_RL.json` экспортирован с другой версии прошивки ODrive (другой набор ключей, нет калибровки Hall-edges в файле) —
   стоит проверить, что на всех 4 платах одна версия firmware.

## Как применить конфиг

```bash
pip install odrive
odrivetool restore-config node1_FR.json   # подключить нужную плату по USB (файл содержит node_id — не перепутать платы!)
odrivetool                                # проверить: odrv0.axis0.controller.config.control_mode
```

Пример ручного выравнивания (после проверки числа полюсов!):

```python
# odrivetool, плата подключена по USB
a = odrv0.axis0
a.controller.config.control_mode = 1          # TORQUE_CONTROL (для no_web)
a.controller.config.input_mode   = 6          # TORQUE_RAMP
a.controller.config.torque_ramp_rate = 10
a.config.watchdog_timeout = 0.2
# a.config.motor.pole_pairs = <проверенное значение>
# a.config.motor.current_control_bandwidth = 1000
odrv0.save_configuration()
```
