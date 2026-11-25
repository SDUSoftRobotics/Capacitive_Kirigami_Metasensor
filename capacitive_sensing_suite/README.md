# Capacitive sensing suite (FDC1004 + ESP32-C6)

Firmware and desktop software for the wireless capacitive sensing module used with the kirigami
metasensors: a DFRobot Beetle ESP32-C6 on a custom PCB with a Texas Instruments FDC1004
capacitance-to-digital converter (two sensors, four inputs, shielded U.FL connections).

```
capacitive_sensing_suite/
├── firmware/fdc1004_v3/fdc1004_v3.ino   ESP32-C6 firmware (Arduino)
├── gui/fdc_studio.py                    FDC Studio: live view, board control, recording, calibration wizard, viewer
├── gui/requirements.txt
├── gui/run_fdc_studio.sh                launcher (macOS / Linux)
├── gui/run_fdc_studio.bat               launcher (Windows)
├── data/                                recordings written by FDC Studio (CSV + metadata JSON)
└── calibration_reports/                 calibration reports and board profiles written by FDC Studio
```

## 1. Flash the firmware

1. Arduino IDE with the **esp32 core 3.x** (Boards Manager: "esp32" by Espressif).
2. Board: **DFRobot Beetle ESP32-C6** (or *ESP32C6 Dev Module*), **USB CDC On Boot: Enabled**.
3. Open `firmware/fdc1004_v3/fdc1004_v3.ino` and upload. No extra libraries are needed.

At boot the board prints `NOT ZEROED` once: the zero is only set when you ask for it (see below)
and is then kept in flash.

**Wi-Fi.** Data are sent over USB serial and, when Wi-Fi is configured, over Wi-Fi (TCP port 8888 and
HTTP port 80). Set the network from the serial console or FDC Studio with `wifissid <name>` and
`wifipass <password>` (stored on the board), so no credentials need to be written in the code.

## 2. Run FDC Studio

Requires Python 3.9 or newer.

```bash
cd gui
./run_fdc_studio.sh              # macOS / Linux (creates ~/cal-venv on first run)
./run_fdc_studio.sh --simulate   # try it without a board
```

Windows: double-click `gui\run_fdc_studio.bat` (or run it with `--simulate`).

Manual alternative: `pip install -r gui/requirements.txt` then `python gui/fdc_studio.py`.

Tabs: **Live** (both sensors, raw and calibrated values, status per sample, CSV recording),
**Calibration** (step-by-step wizard), **Viewer** (saved recordings and calibration reports),
**Log** (all serial traffic and commands).

## 3. First use

1. Connect the board by USB (close the Arduino Serial Monitor first) and press **Connect**.
2. **Scan pins**: every input with a cable reads clearly higher than a bare pin (about 2–3 pF).
3. **Calibration wizard → step 1** (*Probe empty*, then *Probe with reference*, using one known
   capacitor) → **Use recommended mode**:
   - `diff`: CINa − CINb, sensors up to about 8 pF;
   - `se`: CINa to GND with automatic CAPDAC range, sensors up to about 100 pF.
4. **Zero** with the cables connected and nothing between A and B. The zero is stored on the board.
5. Connect the sensor; the status badge must read **OK**. *AT ±16 pF RAIL* means a loose input,
   the wrong mode for the sensor, or wrong pins (`pins default`).
6. Optional: wizard steps 2–7 calibrate the readings into true pF with reference capacitors
   (linear or quadratic fit, written to the board, report saved in `calibration_reports/`).

## 4. Serial / network commands

Serial at **460800 baud**, line ending "Newline"; the same commands work over TCP port 8888 or
`GET /cmd?c=<command>` over HTTP.

| Command | Effect |
|---|---|
| `status` | Pins, mode, zero, calibration and error counters of both sensors |
| `meas 1\|2\|all` | Which sensors are converted |
| `mode <s> diff\|se` | Measurement mode (clears that sensor's zero and calibration) |
| `pins <s> <a> <b>` / `pins <s> <a>` / `pins default` | Differential pair / single-ended pin / defaults |
| `zero [s]` / `unzero [s]` | Store or forget the baseline |
| `probe <s>` / `scan` | Diagnostics |
| `autorange <s> on\|off`, `capdac <s> 0..31` | Range in `se` mode |
| `offset <s> <pF>` | Chip offset in `diff` mode |
| `gain\|shift\|quad <s> <v>`, `calclear [s]` | Calibration `cal = quad·c² + gain·c + shift` |
| `rate`, `avg`, `bits`, `stream`, `settle 0\|1`, `human on\|off` | Conversion and output settings |
| `fdcreset`, `reg [hex]`, `regw`, `factory` | Maintenance |
| `wifissid`, `wifipass`, `wifiauto`, `wificlear`, `wifi on\|off`, `wifiscan` | Wi-Fi |

Data line (one JSON object per sample):

```json
{"c1_pf":0.12,"c2_pf":-0.03,"c1_cal_pf":0.11,"c2_cal_pf":null,"st1":"ok","st2":"ok","cd1":0,"cd2":0,"n":123,"sensor_ok":true,"millis":456}
```

`cN_pf` = reading minus the stored zero (pF); `cN_cal_pf` = calibrated value (null without calibration);
`stN` = `ok`, `sat` (at the ±16 pF rail), `settle` (range just changed) or `over` (> ~110 pF in `se`).

## License

MIT, see [LICENSE](LICENSE).
