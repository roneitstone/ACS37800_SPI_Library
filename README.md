# 🔌 ACS37800 SPI Library (Modified from SparkFun)

This project is a modified version of the [SparkFun ACS37800 Power Monitor Arduino Library](https://github.com/sparkfun/SparkFun_ACS37800_Power_Monitor_Arduino_Library), adapted to work with the ACS37800KMACTR-015B5-SPI using the **SPI interface** instead of I²C.

---

## ✳️ About this version
- Adapted for SPI communication.
- Removed all I²C-specific functions.
- No error signaling is currently implemented, as the SPI interface does not support the same communication feedback mechanisms.
- Data processing and conversion logic remain unchanged.

---

## ⚙️ Installation
1. Download or clone this repository.
2. Place the folder in your Arduino libraries directory:
3. Restart the Arduino IDE and open one of the examples:
