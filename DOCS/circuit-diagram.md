# MyWeld ESP32-S3 — Circuit Reference

> Board: JC3248W535 · ESP32-S3 N16R8 · Created: 2026-02-26

---

## 1. System Block Diagram

```
                              +13V BATTERY
                                  |
                 +----------------+----------------+
                 |                |                |
            [1A Buck]       [3A Buck]      [10A CC/CV Buck-Boost]
                 |                |                |
              13.5V            5.0V            5.5V / 5A
                 |                |                |
           TC4428 VDD       ESP32-S3          30SQ060 Schottky
          (Gate Driver)    (via P1 5VIN)          |
                 |                |           SUPERCAPS
            16x MOSFET      Controls:        2S2P 3000F
              Gates         IO46 -> TC4428      |
                 |          IO16 -> Charger      |
            16x IXTP        IO14 <- Button       |
            170N075T2       IO5  <- V sense      |
                 |          IO6  <- 13.5V sense  |
                 |          IO7  <- Contact       |
            DRAIN BUS --------- SUPERCAP (+) ----+
                 |
            SOURCE BUS
                 |
            ELECTRODES (+)(-)
                 |
                GND (common)
```

---

## 2. Power Distribution

```
    +13V BATTERY
        |
        +-------> [1A Buck] -------> 13.5V ----> TC4428 VDD (always on)
        |
        +-------> [3A Buck] -------> 5.0V  ----> ESP32 P1 +5VIN (always on)
        |
        +-------> [10A Buck-Boost] -> 5.5V ----> 30SQ060 ----> Supercaps
                       |
                      KEY  <--- 2N2222 Collector
                                    |
                                   Base --- 1k --- IO16 (CHARGER_EN)
                                    |
                                 Emitter
                                    |
                                   GND

    Charger Control Truth Table:
    +-----------+-------------+-------------+-----------+
    | IO16      | 2N2222      | KEY Pin     | Charger   |
    +-----------+-------------+-------------+-----------+
    | LOW       | OFF         | Floating    | ON        |
    | HIGH      | ON          | GND         | OFF       |
    +-----------+-------------+-------------+-----------+
```

---

## 3. Supercap Bank (2S2P)

```
    From 30SQ060 Schottky (+)
        |
        +---+------------------+
        |   |                  |
      [3000F 3V]           [3000F 3V]          Parallel pair (top)
      [  Cap 1 ]           [  Cap 2 ]
        |   |                  |
        +---+------ mid ------+
        |   |                  |
      [3000F 3V]           [3000F 3V]          Parallel pair (bottom)
      [  Cap 3 ]           [  Cap 4 ]
        |   |                  |
        +---+------------------+
        |
       GND

    Bank specs:
      Capacitance : 3000F  (series halves, parallel doubles)
      Voltage     : 0-6.0V (2 x 3.0V series)
      Operating   : max 5.7V (derated)
      Energy @5.7V: 48,735 J
      Peak current: ~2000A
```

---

## 4. Gate Drive Circuit

```
    13.5V (from 1A Buck)
        |
        +--------+---- VDD
        |        |
        |   +---------+
        |   |  TC4428  |
        |   |          |
        |   | IN_A  (2)|<---- IO46 (OUTPUT_PIN)
        |   | IN_B  (4)|<---- IO46 (OUTPUT_PIN, same signal)
        |   |          |
        |   | OUT_A (7)|---+
        |   | OUT_B (5)|---+--- outputs paralleled
        |   |          |   |
        |   | GND   (3)|   |
        |   +---------+   |
        |        |         |
        |       GND        |
        |                  |
        |   1.SKE12CA      |
        |   12V TVS        |
        +---|>|<|----------+--- to MOSFET gates via 10R each
        |   (bidirectional)|
       GND                 |
                           |
             +-------------+---- GATE BUS
             |
             +-- 10R -- Gate Q1  (IXTP170N075T2)
             +-- 10R -- Gate Q2
             +-- 10R -- Gate Q3
             :          :
             +-- 10R -- Gate Q16
```

---

## 5. MOSFET Bank & Welding Output

```
    Supercap (+)
        |
    DRAIN BUS (all 16 drains connected)
        |
    +---+---+---+---+---+--- ... ---+---+
    |   |   |   |   |   |           |   |
    D   D   D   D   D   D           D   D
    |   |   |   |   |   |           |   |
    Q1  Q2  Q3  Q4  Q5  Q6        Q15 Q16   (IXTP170N075T2)
    |   |   |   |   |   |           |   |    75V / 170A / 2.1mR
    S   S   S   S   S   S           S   S
    |   |   |   |   |   |           |   |
    +---+---+---+---+---+--- ... ---+---+
        |
    SOURCE BUS
        |
    WELDING ELECTRODES  (+)  (-)
        |                     |
        +-------[work]--------+
        |
       GND

    Combined Rds(on) = 2.1mR / 16 = ~0.13 mR
    Peak current ~ 2000A (depends on contact resistance)
```

---

## 6. ESP32-S3 I/O Map (JC3248W535)

```
    +----------------------------------------------------------+
    |               JC3248W535  (ESP32-S3 N16R8)                |
    |            16MB Flash QIO  /  8MB PSRAM OPI               |
    |                                                          |
    |  DISPLAY (QSPI, internal)          AUDIO (I2S, internal) |
    |    CLK  = IO47                       LRCLK = IO2         |
    |    CS   = IO45                       BCLK  = IO42        |
    |    TE   = IO38                       DOUT  = IO41        |
    |    D0   = IO21                       -> P6 speaker       |
    |    D1   = IO48                                           |
    |    D2   = IO40                     SD CARD (internal)    |
    |    D3   = IO39                       CLK   = IO12        |
    |    BL   = IO1  (PWM)                 MOSI  = IO11        |
    |                                      MISO  = IO13        |
    |  TOUCH (I2C, internal)               CS    = IO10        |
    |    SCL  = IO8                                            |
    |    SDA  = IO4                                            |
    |    Addr = 0x3B                                           |
    +----------------------------------------------------------+

    P1 Header (UART/Power):
    +------+------+------+------+
    | 5VIN |  TXD |  RXD |  GND |
    +------+------+------+------+

    P2 Header (User I/O):
    +------+------+------+------+------+------+------+------+
    | IO5  | IO6  | IO7  | IO15 | IO16 | IO46 | IO9  | IO14 |
    | pin1 | pin2 | pin3 | pin4 | pin5 | pin6 | pin7 | pin8 |
    +------+------+------+------+------+------+------+------+
      |      |      |      |      |      |      |      |
      V_sns  P_sns  C_det ENC_S1 CHG_EN OUTPUT Spare  BTN

    P3/P4 Headers (duplicated):       P5 (Battery):
    +------+------+------+------+     +------+------+
    |  GND | 3.3V | IO17 | IO18 |     | -VAT | +VAT |
    +------+------+------+------+     +------+------+
                   ENC_S2 ENC_KEY      DO NOT use for 13V!
```

---

## 7. ADC Voltage Dividers

```
    A) SUPERCAP VOLTAGE (IO5 / ADC1_CH4)

        Supercap (+)
            |
           10k  R1
            |
            +-------> IO5
            |
           15k  R2
            |
           GND

        Ratio  = 15 / (10+15) = 0.600
        @5.7V  = 3.42V at ADC (11dB atten)
        Formula: V_real = V_adc x (25/15)


    B) PROTECTION RAIL (IO6 / ADC1_CH5)

        Gate Drive Rail (13.5V nominal, safe up to ~25V)
            |
           100k  R1
            |
            +-------> IO6
            |
           15k  R2
            |
           GND

        Ratio  = 15 / (100+15) = 0.130
        @13.5V = 1.76V at ADC (plenty of headroom)
        @18V   = 2.35V at ADC (safe)
        @24V   = 3.13V at ADC (safe)
        @25.3V = 3.30V at ADC (absolute ceiling)
        Formula: V_real = V_adc x (115/15)


    C) CONTACT DETECT (IO7 / ADC1_CH6)

        Electrode tip
             |
            4.7k  (isolation from welding transients)
             |
             +-------> IO7
             |
            5.1k  (to GND)
             |
            GND

        Apart: IO7 ~ 0V    (no current path, 5.1k pulls to GND)
        Touch: IO7 rises    (supercap V feeds through workpiece → 4.7k/5.1k divider)
        Threshold: 1.5V     (contact_detected = v_contact > 1.5V)

        Divider ratio = 5.1 / (4.7 + 5.1) = 0.520
        @3.0V cap = 1.56V at IO7  (just above threshold — matches LOW_VOLTAGE_BLOCK)
        @5.5V cap = 2.87V at IO7  (safe)
        @5.7V cap = 2.97V at IO7  (safe, well below 3.3V max)

        Note: Original Kasyan used 10k (Arduino 5V tolerant).
              Reduced to 5.1k for ESP32 3.3V ADC headroom.
```

---

## 8. Weld Button

```
        3.3V (internal pull-up)
         |
        [R]  internal pull-up
         |
         +-------> IO14 (START_PIN)
         |
        [NO]  momentary button
         |
        GND

    Released: IO14 = HIGH
    Pressed : IO14 = LOW
    Debounce: 5ms

    SAFETY: Only physical button can fire in MAN mode.
            BLE alone cannot trigger a weld.
            Touchscreen has NO weld button.
```

---

## 9. Pulse Timing

```
    CHARGER_EN   ____/````````````````````````````````````\____
    (IO16)            |<-- charger OFF during pulse -->|

    OUTPUT_PIN   ________/`````\__________/`````\_____________
    (IO46)               | P1  |   T      | P2  |

                    500us  P1     T (pause)  P2    500us
                   settle                         settle

    +--------+---------+-------+
    | Param  | Range   | Step  |
    +--------+---------+-------+
    | P1     | 0-50 ms | 0.5ms |
    | T      | 0-50 ms | 0.5ms |
    | P2     | 0-50 ms | 0.5ms |  (P2=0 means single pulse)
    +--------+---------+-------+

    Example "0.15mm Nickel": P1=5ms, T=8ms, P2=8ms
    Timing engine: ets_delay_us() (microsecond precision)
```

---

## 10. BLE Architecture

```
    ESP32-S3 (NimBLE)                     Android App (Nordic BLE)
    +------------------------+            +------------------------+
    |  Service: 0x1234       |            |  MyWeld-Android        |
    |                        |            |  Jetpack Compose       |
    |  Params  (0x1235) R/W  |<-- bin --> |  Read/Write params     |
    |  Status  (0x1236) R/N  |--- bin --> |  Live dashboard        |
    |  Command (0x1237) W    |<-- bin --- |  OTA, reboot, cal      |
    |                        |            |                        |
    |  PIN Auth: "1234"      |            |                        |
    |  Max tries: 5          |            |                        |
    |  Lockout: 60s          |            |                        |
    +------------------------+            +------------------------+
```

---

## 11. FreeRTOS Tasks

```
    +--- CORE 0 -------------------------+--- CORE 1 -----------------------+
    |                                     |                                  |
    |  UI Task                            |  Welding Task                    |
    |    Priority : 5                     |    Priority : 10 (HIGHEST)       |
    |    Stack    : 20 KB                 |    Stack    : 4 KB               |
    |    Job      : LVGL render, touch    |    Job      : State machine,     |
    |                                     |              pulse gen,          |
    |  BLE Task                           |              protection checks   |
    |    Priority : 1                     |                                  |
    |    Stack    : 4 KB                  |  ADC Task                        |
    |    Job      : NimBLE GATT,          |    Priority : 3                  |
    |              params, OTA            |    Stack    : 4 KB               |
    |                                     |    Job      : Voltage sampling   |
    |                                     |              every 500ms         |
    |                                     |                                  |
    |                                     |  Audio Task                      |
    |                                     |    Priority : 2                  |
    |                                     |    Stack    : 4 KB               |
    |                                     |    Job      : I2S tone output    |
    +-------------------------------------+----------------------------------+

    Design: Welding @ Core1 Priority10 = never interrupted during pulse.
```

---

## 12. Weld Cycle Signal Flow

```
    Button pressed (IO14 = LOW)
        |
        v
    Check protection rail (IO6) -- FAIL --> Block + error tone
        |
        OK
        v
    Check supercap voltage (IO5) -- < 3.0V --> Block + warning
        |
        OK (> 3.0V)
        v
    CHARGER_EN = HIGH (IO16) -----> 2N2222 ON --> Charger OFF
        |
    wait 500us (settle)
        |
        v
    OUTPUT_PIN = HIGH (IO46) -----> TC4428 ---10R--> 16x MOSFET ON
        |                                           Supercap -> Electrodes
    wait P1 (us precision)                          ~2000A pulse
        |
        v
    OUTPUT_PIN = LOW ----------------> MOSFETs OFF
        |
    wait T (pause)
        |
        v
    OUTPUT_PIN = HIGH ---------------> MOSFETs ON (2nd pulse)
        |
    wait P2
        |
        v
    OUTPUT_PIN = LOW ----------------> MOSFETs OFF
        |
    wait 500us (settle)
        |
        v
    CHARGER_EN = LOW (IO16) ------> 2N2222 OFF --> Charger ON
        |
        v
    playWeldFire() --> I2S --> Speaker
    weldCount++
    Update UI voltage bar
    BLE status notify --> Android app
```

---

## 13. Safety Interlocks Summary

```
    +----+---------------------------+------------+------------------+
    | #  | Check                     | Threshold  | Action           |
    +----+---------------------------+------------+------------------+
    | 1  | Supercap voltage LOW      | < 4.0V     | UI warning       |
    | 2  | Supercap voltage CRITICAL | < 3.0V     | Block welding    |
    | 3  | Gate drive rail LOW       | < 10V      | Block welding    |
    | 4  | Gate drive rail HIGH      | > 18V      | Block welding    |
    | 5  | Gate drive not connected  | < 8V       | Bench mode (OK)  |
    | 6  | OTA update in progress    | --         | Block welding    |
    | 7  | IO46 at boot              | --         | Forced LOW       |
    | 8  | IO16 at boot              | --         | Forced LOW       |
    | 9  | MAN mode trigger          | --         | Physical btn only|
    | 10 | Confirmation time         | 1500ms     | Debounce faults  |
    +----+---------------------------+------------+------------------+
```

---

## 14. Component BOM

| #  | Component              | Part / Value        | Qty | Notes                         |
|----|------------------------|---------------------|-----|-------------------------------|
| 1  | MCU Board              | JC3248W535          | 1   | ESP32-S3 + display + touch    |
| 2  | MOSFET                 | IXTP170N075T2       | 16  | 75V 170A 2.1mΩ TO-220        |
| 3  | Gate Driver            | TC4428              | 1   | Dual channel, paralleled      |
| 4  | Gate Resistors         | 10Ω                 | 16  | One per MOSFET gate           |
| 5  | TVS Diode              | 1.SKE12CA           | 1   | 12V bidirectional             |
| 6  | Charger Module         | 10A CC/CV Buck-Boost| 1   | Set 5.5V / 5A                |
| 7  | Schottky Diode         | 30SQ060             | 1   | 60V 30A reverse blocking      |
| 8  | Transistor             | 2N2222              | 1   | KEY pin isolation             |
| 9  | Base Resistor          | 1kΩ                 | 1   | IO16 to 2N2222 base          |
| 10 | Logic Buck             | 3A Buck Module      | 1   | 13V → 5.0V for ESP32         |
| 11 | Gate Buck              | 1A Buck Module      | 1   | 13V → 13.5V for TC4428       |
| 12 | Supercaps              | 3.0V 3000F          | 4   | 2S2P configuration           |
| 13 | V-Divider R1           | 10kΩ                | 1   | Supercap sense                |
| 14 | V-Divider R2           | 15kΩ                | 1   | Supercap sense                |
| 15 | P-Divider R1           | 100kΩ               | 1   | Protection rail sense         |
| 16 | P-Divider R2           | 15kΩ                | 1   | Protection rail sense         |
| 17 | Contact Isolation      | 4.7kΩ               | 1   | Electrode tip to IO7          |
| 18 | Contact Pull-down      | 5.1kΩ               | 1   | IO7 to GND                    |
| 19 | Battery                | 13V Lead-Acid       | 1   | External power                |
| 20 | Weld Button            | Momentary NO        | 1   | Physical trigger              |
| 21 | Speaker                | 8Ω small            | 1   | Connects to P6                |
| 22 | Rotary Encoder         | KY-040 module       | 1   | 5-pin: 5V, S1(IO15), S2(IO17), KEY(IO18), GND |

---

> ⚠️ **DANGER:** ~2000A peak / ~49kJ stored. Discharge supercaps before servicing.
