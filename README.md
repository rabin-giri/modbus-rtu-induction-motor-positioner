# VFD Cyclic Controller

A robust, Arduino-based industrial motor controller for Variable Frequency Drives (VFDs) using Modbus RTU. This project features a custom cyclic state machine, an interactive LCD/rotary encoder interface, and mathematically generated S-Curve acceleration profiles for ultra-smooth mechanical transitions.

Created by **Rabin**  (2022)

## ✨ Features

* **Mathematical S-Curve Ramping:** Calculates smooth sinusoidal acceleration and deceleration in real-time, eliminating mechanical jerk and protecting gearboxes/couplings.
* **Modbus RTU Control:** Directly commands VFD speed, forward/reverse, and stop states over RS485 without relying on analog voltage signals.
* **Non-Blocking State Machine:** Concurrent execution of background Modbus communication, UI updates, and ramp calculations.
* **HMI Interface:** Easy configuration of target rotations and cycle counts using a 20x4 I2C LCD and a rotary encoder.
* **Cyclic Automation:** Automatically executes repetitive run-stop cycles based on user-defined parameters.

## 🛠️ Hardware Requirements

* **Microcontroller:** Arduino Uno, Nano, or Mega.
* **Display:** 20x4 LCD with I2C module (Address `0x27`).
* **Input:** Rotary Encoder module & 3x Momentary Push Buttons.
* **Communication:** TTL to RS485 module (for Arduino to VFD Modbus connection).
* **VFD:** Any standard Modbus RTU compatible Variable Frequency Drive (Default Slave ID: `0x01`). Tested with INVT drives.

## 🔌 Pin Configuration

| Component | Arduino Pin | Notes |
| :--- | :--- | :--- |
| **Rotary Encoder (Pin A)** | `D12` | |
| **Rotary Encoder (Pin B)** | `D13` | |
| **Start / Stop Button** | `A0 (D14)` | Pulled to GND when pressed |
| **Set Rotation Button** | `D2` | Pulled to GND when pressed |
| **Set Cycle Button** | `D3` | Pulled to GND when pressed |
| **Run Indicator LED** | `D11` | Active HIGH |
| **Modbus RS485 TX/RX** | `D1 / D0` | Hardware Serial |
| **I2C LCD (SDA / SCL)** | `A4 / A5` | Standard Arduino I2C pins |

## ⚠️ Important VFD Configuration

Because this controller mathematically calculates the acceleration and deceleration curves (S-Curve) and sends real-time speed updates over Modbus, **you must disable the VFD's internal ramping parameters.**

1. Navigate to your VFD's acceleration time parameter (e.g., `P00.11` on INVT).
2. Set it to the lowest possible value (e.g., `0.1s`).
3. Navigate to your VFD's deceleration time parameter (e.g., `P00.12`).
4. Set it to the lowest possible value (e.g., `0.1s`).
5. Ensure the VFD Modbus address is set to `0x01` and the baud rate matches `115200`.

*If you leave the VFD's internal ramps at their default settings (e.g., 10 seconds), the VFD will "fight" the Arduino's speed commands, causing lag and stuttering.*

## 💻 Software & Libraries

This project requires the Arduino IDE and the following external library:
* **LiquidCrystal_I2C**: For the 20x4 display. (Install via Arduino Library Manager).

## 🚀 Installation & Usage

1. Clone or download this repository.
2. Open the `.ino` file in the Arduino IDE.
3. Verify that the `LiquidCrystal_I2C` library is installed.
4. Compile and upload to your Arduino.
5. On boot, the LCD will display the Home Screen.
6. **To set parameters:** Press the "Set Rotation" or "Set Cycle" buttons, use the rotary encoder to adjust the values, and press the button again to save.
7. **To start:** Press the "Start/Stop" button. The controller will execute the S-Curve acceleration and begin the cycle. Press again at any time to trigger an Emergency Stop.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details. 

