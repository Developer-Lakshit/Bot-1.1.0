# Competitive Line Follower Bot - Circuit Connections

This document outlines the wiring for the line follower bot based on the Arduino Nano, QTR-8RC sensor, TB6612FNG motor driver, and SSD1306 OLED display.

## **Power Distribution**
* **Battery (e.g., 7.4V LiPo):** Connects to the input of the MP1584 Buck Module and the `VM` pin of the TB6612FNG motor driver.
* **MP1584 Buck Module:** Adjust output to **5.0V**. This 5V output powers the Arduino Nano (`5V` pin), QTR-8RC sensor (`VCC`), SSD1306 OLED (`VCC`), and the logic power of the TB6612FNG (`VCC`).
* **Common Ground:** All `GND` pins from the battery, buck module, Arduino, sensors, and motor driver must be connected together.

## **Component Wiring Table**

| Component | Pin Name | Connects To (Arduino Nano Pin / Other) | Description |
| :--- | :--- | :--- | :--- |
| **QTR-8RC Sensor** | VCC | 5V (from Buck Module) | Sensor Power |
| | GND | GND (Common Ground) | Ground |
| | Sensor 1 (Left) | D2 | Digital Output 1 |
| | Sensor 2 | D3 | Digital Output 2 |
| | Sensor 3 | D4 | Digital Output 3 |
| | Sensor 4 | D5 | Digital Output 4 |
| | Sensor 5 | D6 | Digital Output 5 |
| | Sensor 6 | D7 | Digital Output 6 |
| | Sensor 7 | D8 | Digital Output 7 |
| | Sensor 8 (Right)| D9 | Digital Output 8 |
| **TB6612FNG Driver**| VM | Battery Positive (+) | Motor Power Supply |
| | VCC | 5V (from Buck Module) | Logic Power Supply |
| | GND | GND (Common Ground) | Ground |
| | STBY | 5V (from Buck Module) | Keep HIGH to enable driver |
| | PWMA | D10 | Left Motor Speed (PWM) |
| | AIN1 | A0 | Left Motor Direction 1 |
| | AIN2 | A1 | Left Motor Direction 2 |
| | PWMB | D11 | Right Motor Speed (PWM) |
| | BIN1 | A2 | Right Motor Direction 1 |
| | BIN2 | A3 | Right Motor Direction 2 |
| | AO1, AO2 | Left Motor Terminals | Connect to Left N20 Motor |
| | BO1, BO2 | Right Motor Terminals | Connect to Right N20 Motor |
| **SSD1306 OLED** | VCC | 5V (from Buck Module) | Display Power |
| | GND | GND (Common Ground) | Ground |
| | SCL | A5 | I2C Clock |
| | SDA | A4 | I2C Data |
| **Indicator LED** | Anode (+) | D12 | "End Box" Indicator |
| | Cathode (-) | GND (via 220Ω-330Ω resistor)| Current Limiting Resistor |
| **Control Buttons**| **Output** | **A6** | **See "Analog Button Ladder" below** |

---

## **SPECIAL: Analog Button Ladder (Connects to Pin A6)**

To save digital pins, we use three push-buttons connected to a single analog pin (A6) using a resistor network.

**Components Needed:**
* 3x Push-to-on buttons (Up, Down, Select)
* 1x 1kΩ Resistor
* 1x 2.2kΩ Resistor
* 1x 4.7kΩ Resistor
* 1x 10kΩ Resistor (Pull-down)

**Wiring Diagram:**
+5V
   |
   +-------[ Button UP ]-------[ 1kΩ ]-------+
   |                                         |
   +-------[ Button DOWN ]-----[ 2.2kΩ ]-----+
   |                                         |
   +-------[ Button SELECT ]---[ 4.7kΩ ]-----+
                                             |
                                             +------> To Arduino Pin A6
                                             |
                                         [ 10kΩ ]
                                             |
                                            GND