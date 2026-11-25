# Capacitive Sensing Suite

Firmware and desktop software for a wireless capacitive sensing module based on the
Texas Instruments FDC1004 capacitance-to-digital converter and the DFRobot Beetle ESP32-C6.

## Contents

- `firmware/` – Arduino firmware for the ESP32-C6
- `gui/` – FDC Studio, a Python application for live data, recording and calibration

## Getting started

### Firmware
1. Install the Arduino IDE and the ESP32 board package (esp32 core 3.x).
2. Select **DFRobot Beetle ESP32-C6** and enable **USB CDC On Boot**.
3. Open `firmware/fdc1004_v3/fdc1004_v3.ino` and upload it to the board.

### FDC Studio
Requires Python 3.9 or newer.

```bash
cd gui
pip install -r requirements.txt
python fdc_studio.py
```

Use `python fdc_studio.py --simulate` to try the software without a board.

## Communication

The board streams data as JSON lines over USB serial (460800 baud) and, optionally, over Wi-Fi.
Send `help` to the board to see the available commands.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
