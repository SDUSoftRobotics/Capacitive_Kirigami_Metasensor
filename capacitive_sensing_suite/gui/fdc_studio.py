#!/usr/bin/env python3
"""FDC Studio: GUI for the FDC1004 board.

Live view, board control, calibration wizard and a viewer for saved files.

    pip install -r requirements.txt
    python fdc_studio.py              # pick the port and connect
    python fdc_studio.py --simulate   # no board needed


Tabs:
  Live         both sensors, raw (zeroed) + calibrated, status per sample, CSV recording
  Calibration  wizard: mode finder (diff/se), zero, reference caps, linear/quadratic fit,
               write to board, verify, save report + profile
  Viewer       open recordings (data/*.csv) and calibration reports/profiles
  Log          everything the board sent + the commands
"""
import argparse
import collections
import csv
import datetime
import json
import math
import os
import queue
import random
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:                      # the simulator still works
    serial = None

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

BAUD = 460800
TCP_PORT = 8888
HERE = Path(__file__).resolve().parent
OUT_DIR = HERE.parent / "data"
REPORT_DIR = HERE.parent / "calibration_reports"
SENSORS = (1, 2)
CH_NAME = {1: "c12", 2: "c34"}                      # names used by capacitive_system / profiles
COLORS = {1: "#1f77b4", 2: "#d95f02"}
ST_COLOR = {"ok": "#2e7d32", "sat": "#c62828", "over": "#c62828", "settle": "#ef6c00", "nodata": "#757575",
            "off": "#9e9e9e", "?": "#757575"}
ST_TEXT = {"ok": "OK", "sat": "AT ±16 pF RAIL", "over": "ABOVE RANGE", "settle": "re-ranging", "nodata": "no data",
           "off": "not measured", "?": "-"}


def now_iso():
    return datetime.datetime.now().isoformat(timespec="seconds")


# --- links: serial, TCP (wifi), simulator. Each one puts parsed JSON dicts in a queue ---
class _LineReader(threading.Thread):
    def __init__(self, q):
        super().__init__(daemon=True)
        self.q, self.alive = q, True

    def _emit_lines(self, buf):
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("{"):
                try:
                    self.q.put(json.loads(line))
                    continue
                except ValueError:
                    pass
            self.q.put({"event": "text", "text": line})
        return buf


class SerialLink(_LineReader):
    def __init__(self, port, q):
        super().__init__(q)
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self.lock = threading.Lock()
        self.name = port
        self.usb_serial = ""
        for p in serial.tools.list_ports.comports():
            if p.device == port:
                self.usb_serial = p.serial_number or ""

    def send(self, text):
        with self.lock:
            try:
                self.ser.write((text + "\n").encode())
            except Exception as e:
                self.q.put({"event": "disconnected", "text": str(e)})

    def close(self):
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass

    def run(self):
        buf = b""
        while self.alive:
            try:
                buf += self.ser.read(4096)
            except Exception as e:
                if self.alive:
                    self.q.put({"event": "disconnected", "text": str(e)})
                return
            buf = self._emit_lines(buf)


class TcpLink(_LineReader):
    def __init__(self, host, q):
        super().__init__(q)
        self.sock = socket.create_connection((host, TCP_PORT), timeout=6)
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.name = f"wifi://{host}:{TCP_PORT}"
        self.usb_serial = ""

    def send(self, text):
        with self.lock:
            try:
                self.sock.sendall((text + "\n").encode())
            except OSError as e:
                self.q.put({"event": "disconnected", "text": str(e)})

    def close(self):
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass

    def run(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("the board closed the connection")
                buf += chunk
            except socket.timeout:
                continue
            except Exception as e:
                if self.alive:
                    self.q.put({"event": "disconnected", "text": str(e)})
                return
            buf = self._emit_lines(buf)


class SimLink(threading.Thread):
    """Fake v3 board for testing. The 'virtual capacitor' between A and B is set from the GUI.
    In diff mode it reads -2x the cap, like the real board did during calibration."""

    def __init__(self, q):
        super().__init__(daemon=True)
        self.q, self.alive = q, True
        self.name, self.usb_serial = "simulator", "SIM:00:00:00"
        self.lock = threading.Lock()
        self.cg = [4.0, 3.5, 2.5, 2.6]          # pin to GND
        self.cm = {1: 0.0, 2: 0.0}               # virtual capacitor between A and B (set by the GUI)
        self.cfg = dict(rate=400, avg=1, bits=24, stream_ms=10, i2c_khz=400, meas=3, settle=0, human=0)
        self.s = {i: dict(mode="diff", a=2 * i - 2, b=2 * i - 1, capdac=0, auto=True, off=0.0, zero=0.0, zeroed=False,
                          a0=0.0, a1=1.0, a2=0.0, cal=False) for i in SENSORS}
        self.n = 0

    def _measured(self, i):
        s = self.s[i]
        cm = self.cm[i]
        if s["mode"] == "diff":
            v = self.cg[s["a"]] - self.cg[s["b"]] - 2.0 * cm - s["off"] + random.gauss(0, 0.004)
            return (max(-16.0, min(15.9999, v)), abs(v) >= 15.99)
        v = self.cg[s["a"]] + cm + random.gauss(0, 0.004)
        s["capdac"] = max(0, min(31, int((v - 2) // 3.125)))
        return (min(v, 112.0), v >= 112)

    def _data(self):
        out = {"c1_pf": None, "c2_pf": None, "c1_cal_pf": None, "c2_cal_pf": None}
        for i in SENSORS:
            on = self.cfg["meas"] & (1 << (i - 1))
            if not on:
                out[f"st{i}"] = "off"
            else:
                m, sat = self._measured(i)
                v = m - self.s[i]["zero"]
                s = self.s[i]
                out[f"c{i}_pf"] = round(v, 5)
                out[f"c{i}_cal_pf"] = round(s["a2"] * v * v + s["a1"] * v + s["a0"], 5) if s["cal"] else None
                out[f"st{i}"] = ("over" if s["mode"] == "se" else "sat") if sat else "ok"
            out[f"cd{i}"] = self.s[i]["capdac"]
        self.n += 1
        out.update(n=self.n, sensor_ok=True, millis=int((time.perf_counter() - self.t0) * 1000))
        return out

    def _config(self):
        d = {"event": "config", "fw": "two_channels", "fw3": True, "ver": "3.0-sim", **self.cfg,
             "shared": False, "out_hz": 100.0, "i2c_err": 0, "timeouts": 0, "resets": 0,
             "wifi": 0, "ssid": "", "ip": "", "wifi_auto": False, "tcp_port": TCP_PORT}
        for i in SENSORS:
            s = self.s[i]
            d[f"pins{i}"] = f"{s['a'] + 1}" if s["mode"] == "se" else f"{s['a'] + 1}-{s['b'] + 1}"
            d.update({f"mode{i}": s["mode"], f"capdac{i}": s["capdac"], f"autorange{i}": s["auto"],
                      f"offset{i}_pf": s["off"], f"zero{i}_pf": s["zero"], f"zeroed{i}": s["zeroed"],
                      f"gain{i}": s["a1"], f"shift{i}": s["a0"], f"quad{i}": s["a2"], f"cal{i}": s["cal"]})
        return d

    def _msg(self, t):
        self.q.put({"event": "msg", "text": t})

    def send(self, text):
        with self.lock:
            self._command(text.strip().lower())
            self.q.put(self._config())

    def _sel(self, tok):
        return {"1": [1], "2": [2], "all": [1, 2], "both": [1, 2]}.get(tok, [])

    def _command(self, line):
        t = line.split()
        if not t:
            return
        c = t[0]
        arg = t[1] if len(t) > 1 else ""
        if c in ("zero", "cal", "c"):
            for i in self._sel(arg or "all"):
                s = self.s[i]
                s["off"] = 0.0
                m, sat = self._measured(i)
                if sat:
                    self._msg(f"S{i}: ERR the reading is at the +-16 pF rail")
                    continue
                if s["mode"] == "diff":
                    s["off"], s["zero"] = m, 0.0
                else:
                    s["zero"] = m
                s["zeroed"] = True
                self._msg(f"S{i}: OK zero {m:.4f} pF")
        elif c == "unzero":
            for i in self._sel(arg or "all"):
                self.s[i].update(zero=0.0, off=0.0, zeroed=False)
            self._msg("OK zero cleared")
        elif c == "mode" and len(t) >= 3:
            for i in self._sel(arg):
                self.s[i].update(mode=t[2], zero=0.0, off=0.0, zeroed=False, a0=0.0, a1=1.0, a2=0.0, cal=False)
            self._msg(f"OK mode {arg} {t[2]}. Zero and calibration cleared")
        elif c == "pins" and arg == "default":
            for i in SENSORS:
                self.s[i].update(mode="diff", a=2 * i - 2, b=2 * i - 1, zeroed=False, zero=0.0, off=0.0, cal=False)
            self._msg("OK pins default")
        elif c == "meas":
            self.cfg["meas"] = {"1": 1, "2": 2, "all": 3, "both": 3}.get(arg, 3)
            self._msg(f"OK meas {arg}")
        elif c in ("gain", "shift", "quad") and len(t) >= 3:
            for i in self._sel(arg):
                self.s[i][{"gain": "a1", "shift": "a0", "quad": "a2"}[c]] = float(t[2])
                self.s[i]["cal"] = True
            self._msg(f"OK {c} {arg} {t[2]}")
        elif c == "calclear":
            for i in self._sel(arg or "all"):
                self.s[i].update(a0=0.0, a1=1.0, a2=0.0, cal=False)
            self._msg("OK calibration cleared")
        elif c == "probe":
            i = int(arg or 1)
            s = self.s[i]
            a, b = 2 * i - 2, 2 * i - 1
            diff = self.cg[a] - self.cg[b] - 2 * self.cm[i]
            self.q.put({"event": "probe", "s": i, "a": a + 1, "b": b + 1, "diff_pf": round(max(-16, min(15.999, diff)), 4),
                        "diff_sat": abs(diff) >= 15.99, "se_a_pf": round(self.cg[a] + self.cm[i], 4),
                        "se_b_pf": round(self.cg[b] + self.cm[i], 4)})
            self._msg(f"probe S{i} done")
        elif c == "scan":
            self.q.put({"event": "scan", **{f"cin{p + 1}_pf": round(self.cg[p], 3) for p in range(4)}})
            self._msg("scan done")
        elif c in ("rate", "avg", "stream", "settle", "i2c", "bits"):
            key = {"stream": "stream_ms", "i2c": "i2c_khz"}.get(c, c)
            self.cfg[key] = int(float(arg or 0))
            self._msg(f"OK {c} {arg}")
        elif c == "factory":
            self.__init__(self.q)
        elif c in ("get", "status", "help"):
            self._msg("OK (simulator)")
        else:
            self._msg("ERR unknown command (simulator)")

    def close(self):
        self.alive = False

    def run(self):
        self.t0 = time.perf_counter()
        self._msg("Simulator: firmware v3. Use the 'Virtual capacitor' boxes to plug things in.")
        self.q.put(self._config())
        while self.alive:
            time.sleep(0.01)
            with self.lock:
                self.q.put(self._data())


# --- ring buffer for the plots ---
class Ring:
    def __init__(self, n=200_000):
        self.n, self.i, self.full = n, 0, False
        self.t = np.zeros(n)
        self.v = {i: np.full(n, np.nan) for i in SENSORS}
        self.c = {i: np.full(n, np.nan) for i in SENSORS}

    def append(self, t, vals, cals):
        k = self.i
        self.t[k] = t
        for i in SENSORS:
            self.v[i][k] = vals[i]
            self.c[i][k] = cals[i]
        self.i = (k + 1) % self.n
        self.full |= self.i == 0

    def last(self, seconds):
        if not self.full and self.i == 0:
            return None
        idx = np.r_[self.i:self.n, 0:self.i] if self.full else np.arange(self.i)
        t = self.t[idx]
        keep = t >= t[-1] - seconds
        idx = idx[keep]
        return self.t[idx], {i: self.v[i][idx] for i in SENSORS}, {i: self.c[i][idx] for i in SENSORS}

    def clear(self):
        self.__init__(self.n)


# --- CSV recorder (same columns as capacitive_system.py + status/capdac) ---
class Recorder:
    COLS = ["Time_s", "Datetime", "Board_ms", "C12_raw_pF", "C34_raw_pF", "C12_cal_pF", "C34_cal_pF",
            "S1_status", "S2_status", "S1_capdac", "S2_capdac"]

    def __init__(self, path, meta):
        self.path = path
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(self.COLS)
        self.n = 0
        with open(str(path).replace(".csv", "_meta.json"), "w") as m:
            json.dump(meta, m, indent=2)

    def write(self, t, ms, vals, cals, sts, cds):
        f = lambda x: "NaN" if x is None or not math.isfinite(x) else f"{x:.6f}"
        self.w.writerow([f"{t:.4f}", datetime.datetime.now().isoformat(sep=" ", timespec="milliseconds"), ms,
                         f(vals[1]), f(vals[2]), f(cals[1]), f(cals[2]), sts[1], sts[2], cds[1], cds[2]])
        self.n += 1

    def close(self):
        self.f.close()


# --- calibration math ---
def fit_poly(meas, true, degree):
    """true = a2*m^2 + a1*m + a0. Returns (a0, a1, a2, residuals)."""
    m, t = np.asarray(meas, float), np.asarray(true, float)
    deg = min(degree, len(m) - 1)
    coef = np.polyfit(m, t, deg)                    # highest power first
    a2 = coef[-3] if deg >= 2 else 0.0
    a1 = coef[-2] if deg >= 1 else 0.0
    a0 = coef[-1]
    res = t - (a2 * m * m + a1 * m + a0)
    return float(a0), float(a1), float(a2), res


# --- calibration wizard ---
class Wizard(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.points = []              # dict(kind, true_pf, measured_pf, noise_ff, n)
        self.fit = None
        self.collect = None           # active measurement: dict(sensor, until, vals, bad, done)
        self.probe_empty = None
        self.probe_ref = None
        self.verify = []

        lay = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(left)
        scroll.setMinimumWidth(470)
        lay.addWidget(scroll, 0)

        def box(title):
            g = QtWidgets.QGroupBox(title)
            v = QtWidgets.QVBoxLayout(g)
            lv.addWidget(g)
            return v

        # step 0
        v = box("0 · Sensor")
        h = QtWidgets.QHBoxLayout()
        self.cmbSensor = QtWidgets.QComboBox()
        self.cmbSensor.addItem("Sensor 1  (Sensor1A / Sensor1B)", 1)
        self.cmbSensor.addItem("Sensor 2  (Sensor2A / Sensor2B)", 2)
        h.addWidget(self.cmbSensor)
        self.chkOnly = QtWidgets.QCheckBox("measure only this sensor")
        self.chkOnly.setChecked(True)
        self.chkOnly.setToolTip("Sends 'meas 1' / 'meas 2': no crosstalk from the other input, higher rate.")
        h.addWidget(self.chkOnly)
        v.addLayout(h)
        self.spnSecs = QtWidgets.QDoubleSpinBox()
        self.spnSecs.setRange(1, 60)
        self.spnSecs.setValue(5)
        self.spnSecs.setSuffix(" s per point")
        v.addWidget(self.spnSecs)
        b = QtWidgets.QPushButton("Start: clear calibration on the board for this sensor")
        b.clicked.connect(self.start)
        v.addWidget(b)

        # step 1
        v = box("1 · Mode finder (optional, recommended once per sensor type)")
        v.addWidget(self._lbl("Finds which measurement follows a capacitor between A and B. "
                              "Leave A/B empty → <b>Probe empty</b>. Then connect one reference capacitor "
                              "between A and B → <b>Probe with reference</b>."))
        h = QtWidgets.QHBoxLayout()
        self.spnProbeC = QtWidgets.QDoubleSpinBox()
        self.spnProbeC.setRange(0.1, 100)
        self.spnProbeC.setValue(4.7)
        self.spnProbeC.setSuffix(" pF")
        b1 = QtWidgets.QPushButton("Probe empty")
        b1.clicked.connect(lambda: self.probe("empty"))
        b2 = QtWidgets.QPushButton("Probe with reference")
        b2.clicked.connect(lambda: self.probe("ref"))
        h.addWidget(b1)
        h.addWidget(self.spnProbeC)
        h.addWidget(b2)
        v.addLayout(h)
        self.lblProbe = self._lbl("")
        v.addWidget(self.lblProbe)
        self.btnUseMode = QtWidgets.QPushButton("Use recommended mode")
        self.btnUseMode.setEnabled(False)
        self.btnUseMode.clicked.connect(self.use_mode)
        v.addWidget(self.btnUseMode)

        # step 2
        v = box("2 · Zero")
        v.addWidget(self._lbl("Disconnect the sensor / capacitor from A and B (cables stay as they will be used), "
                              "then zero. The zero is stored on the board and is <b>not</b> redone at power-up."))
        b = QtWidgets.QPushButton("Zero now (nothing connected)")
        b.clicked.connect(self.zero)
        v.addWidget(b)
        b = QtWidgets.QPushButton("Measure the 0 pF point")
        b.clicked.connect(lambda: self.measure("baseline", 0.0))
        v.addWidget(b)

        # step 3
        v = box("3 · Reference capacitors")
        v.addWidget(self._lbl("Connect a capacitor of known value between A and B, enter its value, Measure. "
                              "Use at least 3 values spread over the sensor's range. Samples that are not "
                              "'ok' (rail, re-ranging) make the point invalid."))
        h = QtWidgets.QHBoxLayout()
        self.spnTrue = QtWidgets.QDoubleSpinBox()
        self.spnTrue.setRange(0, 200)
        self.spnTrue.setDecimals(3)
        self.spnTrue.setValue(2.2)
        self.spnTrue.setSuffix(" pF")
        h.addWidget(self.spnTrue)
        b = QtWidgets.QPushButton("Measure")
        b.clicked.connect(lambda: self.measure("reference", self.spnTrue.value()))
        h.addWidget(b)
        v.addLayout(h)
        self.prog = QtWidgets.QProgressBar()
        self.prog.setRange(0, 100)
        v.addWidget(self.prog)
        self.tbl = QtWidgets.QTableWidget(0, 5)
        self.tbl.setHorizontalHeaderLabels(["kind", "true pF", "measured pF", "noise fF", "residual pF"])
        self.tbl.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tbl.setMinimumHeight(170)
        v.addWidget(self.tbl)
        b = QtWidgets.QPushButton("Delete selected point")
        b.clicked.connect(self.delete_point)
        v.addWidget(b)

        # step 4
        v = box("4 · Fit")
        h = QtWidgets.QHBoxLayout()
        self.cmbDeg = QtWidgets.QComboBox()
        self.cmbDeg.addItem("linear  true = gain·m + shift", 1)
        self.cmbDeg.addItem("quadratic  + quad·m²", 2)
        h.addWidget(self.cmbDeg)
        b = QtWidgets.QPushButton("Fit")
        b.clicked.connect(self.do_fit)
        h.addWidget(b)
        v.addLayout(h)
        self.lblFit = self._lbl("")
        v.addWidget(self.lblFit)

        # step 5-7
        v = box("5 · Write  ·  6 · Verify  ·  7 · Save")
        b = QtWidgets.QPushButton("Write gain / shift / quad to the board")
        b.clicked.connect(self.write)
        v.addWidget(b)
        h = QtWidgets.QHBoxLayout()
        self.spnVer = QtWidgets.QDoubleSpinBox()
        self.spnVer.setRange(0, 200)
        self.spnVer.setDecimals(3)
        self.spnVer.setValue(4.7)
        self.spnVer.setSuffix(" pF")
        h.addWidget(self.spnVer)
        b = QtWidgets.QPushButton("Verify with this capacitor")
        b.clicked.connect(lambda: self.measure("verify", self.spnVer.value()))
        h.addWidget(b)
        v.addLayout(h)
        self.lblVer = self._lbl("")
        v.addWidget(self.lblVer)
        b = QtWidgets.QPushButton("Save report (.json + .csv) and update the board profile")
        b.clicked.connect(self.save)
        v.addWidget(b)
        b = QtWidgets.QPushButton("Upload a saved profile / report (.json) to the board")
        b.setToolTip("Writes gain / shift / quad for the selected sensor from a profile_*.json or a "
                     "calibration report. Checks that the board's mode, pins and zero match first.")
        b.clicked.connect(self.upload_profile)
        v.addWidget(b)
        lv.addStretch(1)

        # right: plots
        right = QtWidgets.QVBoxLayout()
        self.pFit = pg.PlotWidget(title="measured → true")
        self.pFit.setLabel("bottom", "measured (zeroed) pF")
        self.pFit.setLabel("left", "true pF")
        self.pFit.showGrid(x=True, y=True, alpha=0.3)
        self.pRes = pg.PlotWidget(title="residual after fit")
        self.pRes.setLabel("bottom", "true pF")
        self.pRes.setLabel("left", "pF")
        self.pRes.showGrid(x=True, y=True, alpha=0.3)
        self.pRes.getAxis("left").enableAutoSIPrefix(False)
        right.addWidget(self.pFit, 3)
        right.addWidget(self.pRes, 2)
        lay.addLayout(right, 1)

    @staticmethod
    def _lbl(t):
        l = QtWidgets.QLabel(t)
        l.setWordWrap(True)
        l.setTextFormat(QtCore.Qt.RichText)
        return l

    @property
    def sensor(self):
        return self.cmbSensor.currentData()

    # --- actions ---
    def _need_link(self):
        if not self.app.link:
            QtWidgets.QMessageBox.information(self, "Not connected", "Connect to the board first.")
            return False
        return True

    def start(self):
        if not self._need_link():
            return
        s = self.sensor
        if self.chkOnly.isChecked():
            self.app.send(f"meas {s}")
        self.app.send(f"calclear {s}")
        self.points, self.fit, self.verify = [], None, []
        self.refresh()
        self.app.say(f"Wizard: calibrating sensor {s}")

    def probe(self, which):
        if not self._need_link():
            return
        if not self.app.fw3:
            QtWidgets.QMessageBox.information(self, "Firmware", "The mode finder needs firmware v3.")
            return
        self.app.pending_probe = (self, which)
        self.app.send(f"probe {self.sensor}")

    def on_probe(self, which, d):
        if which == "empty":
            self.probe_empty = d
        else:
            self.probe_ref = d
        self._show_probe()

    def _show_probe(self):
        e, r = self.probe_empty, self.probe_ref
        if not e:
            self.lblProbe.setText("Empty: waiting.")
            return
        txt = (f"Empty: diff {e['diff_pf']} pF, CIN{e['a']} to GND {e['se_a_pf']} pF, "
               f"CIN{e['b']} to GND {e['se_b_pf']} pF.")
        self.recommend = None
        if r:
            C = self.spnProbeC.value()
            k_d = (r["diff_pf"] - e["diff_pf"]) / C if not r.get("diff_sat") else float("nan")
            k_a = (r["se_a_pf"] - e["se_a_pf"]) / C
            k_b = (r["se_b_pf"] - e["se_b_pf"]) / C
            rng_d = 16.0 / abs(k_d) if k_d and math.isfinite(k_d) else 0
            rng_a = 105.0 / abs(k_a) if abs(k_a) > 0.05 else 0
            txt += (f"<br>With {C:g} pF: sensitivity <b>diff {k_d:+.2f}</b> (range ≈ ±{rng_d:.1f} pF of sensor), "
                    f"<b>se on CIN{e['a']} {k_a:+.2f}</b> (range ≈ {rng_a:.0f} pF), se on CIN{e['b']} {k_b:+.2f}.")
            if abs(k_a) >= 0.5 and rng_a > rng_d:
                self.recommend = "se"
                txt += "<br><b>Recommended: se</b> (single-ended on the A pin, CAPDAC auto-range): larger range."
            elif math.isfinite(k_d) and abs(k_d) >= 0.3:
                self.recommend = "diff"
                txt += "<br><b>Recommended: diff</b>."
            else:
                txt += ("<br>Neither mode responds well. Check the wiring (scan), or connect the B electrode "
                        "to GND and use se.")
        self.lblProbe.setText(txt)
        self.btnUseMode.setEnabled(bool(self.recommend))

    def use_mode(self):
        if self.recommend:
            self.app.send(f"mode {self.sensor} {self.recommend}")
            self.points, self.fit = [], None
            self.refresh()

    def zero(self):
        if self._need_link():
            self.app.send(f"zero {self.sensor}")

    def measure(self, kind, true_pf):
        if not self._need_link() or self.collect:
            return
        self.collect = dict(sensor=self.sensor, kind=kind, true=true_pf, t0=time.time(), skip=0.4,
                            dur=self.spnSecs.value(), raw=[], cal=[], bad=0)

    def feed(self, sample):
        """Called by the app for every data line."""
        c = self.collect
        if not c:
            return
        el = time.time() - c["t0"]
        if el < c["skip"]:
            return
        i = c["sensor"]
        st = sample["st"][i]
        if st != "ok":
            c["bad"] += 1
        elif sample["v"][i] is not None:
            c["raw"].append(sample["v"][i])
            if sample["c"][i] is not None:
                c["cal"].append(sample["c"][i])
        self.prog.setValue(int(100 * min(1, (el - c["skip"]) / c["dur"])))
        if el - c["skip"] >= c["dur"]:
            self.collect = None
            self._finish(c)

    def _finish(self, c):
        n = len(c["raw"])
        if n < 5 or c["bad"] > 0.02 * (n + c["bad"]):
            QtWidgets.QMessageBox.warning(self, "Point rejected",
                                          f"{c['bad']} samples were not 'ok' (rail / re-ranging) and {n} were usable. "
                                          "Check the connection, or the value is out of range for this mode "
                                          "(see the mode finder).")
            return
        raw = np.asarray(c["raw"])
        m, sd = float(np.mean(raw)), float(np.std(raw))
        if c["kind"] == "verify":
            cal = float(np.mean(c["cal"])) if c["cal"] else (self._apply(m) if self.fit else float("nan"))
            self.verify.append(dict(true_pf=c["true"], measured_pf=m, corrected_pf=cal, noise_ff=sd * 1000))
            self.lblVer.setText(f"Verify {c['true']:g} pF: measured {m:.4f}, corrected <b>{cal:.4f} pF</b> "
                                f"(error {cal - c['true']:+.4f} pF)")
            return
        if c["kind"] == "baseline":
            self.points = [p for p in self.points if p["kind"] != "baseline"]
        self.points.append(dict(kind=c["kind"], true_pf=c["true"], measured_pf=m, noise_ff=sd * 1000, n=n))
        self.points.sort(key=lambda p: p["true_pf"])
        self.fit = None
        self.refresh()

    def delete_point(self):
        rows = sorted({i.row() for i in self.tbl.selectedIndexes()}, reverse=True)
        for r in rows:
            del self.points[r]
        self.fit = None
        self.refresh()

    def _apply(self, m):
        a0, a1, a2 = self.fit["a0"], self.fit["a1"], self.fit["a2"]
        return a2 * m * m + a1 * m + a0

    def do_fit(self):
        if len(self.points) < 2:
            QtWidgets.QMessageBox.information(self, "Fit", "Measure at least 2 points (better 0 pF + 3 references).")
            return
        deg = self.cmbDeg.currentData()
        if deg == 2 and not self.app.fw3:
            QtWidgets.QMessageBox.information(self, "Fit", "The v2 firmware stores only gain/shift: using linear.")
            deg = 1
        if deg == 2 and len(self.points) < 4:
            QtWidgets.QMessageBox.information(self, "Fit", "A quadratic fit needs at least 4 points.")
            return
        m = [p["measured_pf"] for p in self.points]
        t = [p["true_pf"] for p in self.points]
        a0, a1, a2, res = fit_poly(m, t, deg)
        self.fit = dict(a0=a0, a1=a1, a2=a2, degree=deg, residuals=res.tolist(),
                        max_residual_pf=float(np.max(np.abs(res))), points=len(m))
        self.lblFit.setText(f"gain {a1:.6f}, shift {a0:+.5f} pF" + (f", quad {a2:+.6g}" if deg == 2 else "") +
                            f"<br>max residual <b>{self.fit['max_residual_pf']:.4f} pF</b> over {len(m)} points")
        self.refresh()

    def write(self):
        if not self.fit or not self._need_link():
            return
        s = self.sensor
        self.app.send(f"gain {s} {self.fit['a1']:.7g}")
        self.app.send(f"shift {s} {self.fit['a0']:.7g}")
        if self.app.fw3:
            self.app.send(f"quad {s} {self.fit['a2']:.7g}")

    def upload_profile(self):
        """Write gain/shift/quad from a saved profile_*.json or *_calibration_*.json to the board."""
        if not self._need_link():
            return
        s = self.sensor
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Calibration profile or report", str(REPORT_DIR),
                                                        "JSON (*.json)")
        if not path:
            return
        try:
            d = json.load(open(path))
        except (OSError, ValueError) as e:
            QtWidgets.QMessageBox.warning(self, "Upload", f"Could not read {path}:\n{e}")
            return
        if "channels" in d:                                   # board profile
            ch = d["channels"].get(CH_NAME[s])
            if not ch:
                QtWidgets.QMessageBox.warning(self, "Upload", f"{os.path.basename(path)} has no calibration for "
                                              f"Sensor {s} ({CH_NAME[s]}).")
                return
            gain, shift, quad = ch["gain"], ch["shift_pf"], ch.get("quad", 0.0)
            mode, pins, zero = ch.get("mode"), ch.get("pins"), ch.get("zero_pf")
            if zero is None and ch.get("source_report"):     # older profiles: zero is in the report
                try:
                    zero = json.load(open(Path(path).parent / ch["source_report"])).get("zero_pf")
                except (OSError, ValueError):
                    pass
        elif "fit" in d:                                      # calibration report
            if d.get("sensor") not in (None, s):
                QtWidgets.QMessageBox.warning(self, "Upload", f"This report is for Sensor {d['sensor']}; "
                                              f"Sensor {s} is selected at the top.")
                return
            f = d["fit"]
            gain, shift, quad = f["gain"], f["shift_pf"], f.get("quad", 0.0)
            mode, pins, zero = d.get("mode"), d.get("pins"), d.get("zero_pf")
        else:
            QtWidgets.QMessageBox.warning(self, "Upload", "Not a profile or calibration report.")
            return

        cfg = self.app.board_cfg
        norm = lambda p: str(p or "").replace("-", " ").strip()
        warn = []
        bmode, bpins, bzero = cfg.get(f"mode{s}"), cfg.get(f"pins{s}"), cfg.get(f"zero{s}_pf")
        if mode and bmode and mode != bmode:
            warn.append(f"mode: file {mode}, board {bmode}")
        if pins and bpins and norm(pins) != norm(bpins):
            warn.append(f"pins: file {pins}, board {bpins}")
        if zero is not None and bzero is not None and abs(zero - bzero) > 0.05:
            warn.append(f"zero: file {zero:.4f} pF, board {bzero:.4f} pF (the fit is only valid for the zero "
                        f"it was measured with)")
        if not self.app.fw3 and quad:
            warn.append("this firmware has no quad command: the quadratic term will be lost")
        text = (f"Sensor {s}: gain {gain:.7g}, shift {shift:+.7g} pF, quad {quad:.7g}\n"
                f"from {os.path.basename(path)}")
        if warn:
            text += "\n\nThe board does not match the file:\n  - " + "\n  - ".join(warn) + "\n\nUpload anyway?"
            ok = QtWidgets.QMessageBox.warning(self, "Upload profile", text,
                                               QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                               QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes
        else:
            ok = QtWidgets.QMessageBox.question(self, "Upload profile", text + "\n\nWrite to the board?") \
                == QtWidgets.QMessageBox.Yes
        if not ok:
            return
        self.app.send(f"gain {s} {gain:.7g}")
        self.app.send(f"shift {s} {shift:.7g}")
        if self.app.fw3:
            self.app.send(f"quad {s} {quad:.7g}")
        self.app.say(f"Uploaded calibration for Sensor {s} from {path}")

    def save(self):
        if not self.fit:
            QtWidgets.QMessageBox.information(self, "Save", "Fit first.")
            return
        s = self.sensor
        cfg = self.app.board_cfg
        board = (self.app.link.usb_serial if self.app.link else "") or "unknown"
        mode = cfg.get(f"mode{s}", "diff")
        pins = str(cfg.get(f"pins{s}", "1-2" if s == 1 else "3-4")).replace("-", " ")
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = board.replace(":", "")
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        base = REPORT_DIR / f"{'C12' if s == 1 else 'C34'}_{mode}_calibration_{tag}_{stamp}"
        pts = [dict(p, corrected_pf=self._apply(p["measured_pf"])) for p in self.points]
        rep = {
            "created": now_iso(), "port": self.app.link.name if self.app.link else "", "board_usb_serial": board,
            "firmware": cfg.get("ver", "?"), "sensor": s, "pins": pins, "mode": mode,
            "pins_label": f"{'CIN' + pins.replace(' ', ' - CIN')} ({'differential' if mode == 'diff' else 'single-ended, CAPDAC auto-range'})",
            "measure_secs": self.spnSecs.value(), "zero_pf": cfg.get(f"zero{s}_pf"),
            "rate": cfg.get("rate"), "avg": cfg.get("avg"),
            "baseline_pf": next((p["measured_pf"] for p in self.points if p["kind"] == "baseline"), None),
            "fit": {"gain": self.fit["a1"], "shift_pf": self.fit["a0"], "quad": self.fit["a2"],
                    "degree": self.fit["degree"], "points": self.fit["points"],
                    "max_residual_pf": self.fit["max_residual_pf"]},
            "equation": "true_pf = quad * m^2 + gain * m + shift_pf   (m = zeroed reading cN_pf)",
            "points": pts + [dict(v, kind="verify") for v in self.verify],
        }
        with open(f"{base}.json", "w") as f:
            json.dump(rep, f, indent=1)
        with open(f"{base}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["true_pf", "measured_pf", "noise_ff", "corrected_pf", "kind"])
            for p in rep["points"]:
                w.writerow([p["true_pf"], p.get("measured_pf"), p.get("noise_ff"), p.get("corrected_pf"), p["kind"]])
        prof_path = REPORT_DIR / f"profile_{tag}.json"
        prof = {"board_usb_serial": board, "equation": rep["equation"], "channels": {}}
        if prof_path.exists():
            try:
                prof = json.load(open(prof_path))
            except ValueError:
                pass
        ms = [p["measured_pf"] for p in self.points]
        prof["channels"][CH_NAME[s]] = {
            "pins": pins, "mode": mode, "gain": self.fit["a1"], "shift_pf": self.fit["a0"], "quad": self.fit["a2"],
            "zero_pf": cfg.get(f"zero{s}_pf"),
            "points": self.fit["points"], "max_residual_pf": self.fit["max_residual_pf"],
            "calibrated_range_measured_pf": [min(ms), max(ms)], "calibrated": rep["created"],
            "source_report": os.path.basename(f"{base}.json")}
        with open(prof_path, "w") as f:
            json.dump(prof, f, indent=2)
        self.app.say(f"Saved {base}.json/.csv and {prof_path}")
        QtWidgets.QMessageBox.information(self, "Saved", f"{base}.json\n{base}.csv\n{prof_path}")

    # --- view ---
    def refresh(self):
        self.tbl.setRowCount(len(self.points))
        res = self.fit["residuals"] if self.fit else [None] * len(self.points)
        for r, (p, e) in enumerate(zip(self.points, res)):
            vals = [p["kind"], f"{p['true_pf']:.3f}", f"{p['measured_pf']:.4f}", f"{p['noise_ff']:.1f}",
                    "" if e is None else f"{e:+.4f}"]
            for c, v in enumerate(vals):
                self.tbl.setItem(r, c, QtWidgets.QTableWidgetItem(v))
        self.pFit.clear()
        self.pRes.clear()
        if not self.points:
            return
        m = np.array([p["measured_pf"] for p in self.points])
        t = np.array([p["true_pf"] for p in self.points])
        self.pFit.plot(m, t, pen=None, symbol="o", symbolBrush=COLORS[self.sensor])
        if self.fit:
            xs = np.linspace(m.min(), m.max(), 100)
            self.pFit.plot(xs, [self._apply(x) for x in xs], pen=pg.mkPen("k", width=1.5))
            self.pRes.plot(t, np.array(self.fit["residuals"]), pen=None, symbol="o", symbolBrush="#c62828")
            self.pRes.addLine(y=0, pen=pg.mkPen("#888", style=QtCore.Qt.DashLine))


# --- viewer: recordings (data/*.csv) and calibration reports/profiles ---
def load_recording(path):
    """Load a recording from the Live tab -> (columns as arrays/lists, meta dict or None)."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        raise ValueError("empty file")
    head, body = rows[0], [r for r in rows[1:] if len(r) == len(rows[0])]
    cols = {}
    for k, name in enumerate(head):
        vals = [r[k] for r in body]
        try:
            cols[name] = np.array([float(v) for v in vals])
        except ValueError:
            cols[name] = vals
    meta = None
    mp = str(path).replace(".csv", "_meta.json")
    if os.path.exists(mp):
        try:
            meta = json.load(open(mp))
        except ValueError:
            pass
    return cols, meta


def _stats(y):
    y = y[np.isfinite(y)]
    if len(y) < 2:
        return None
    return dict(n=len(y), mean=float(y.mean()), sd=float(y.std(ddof=1)), lo=float(y.min()), hi=float(y.max()))


class Viewer(QtWidgets.QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.rec = None              # (t, cols, meta, path)
        lay = QtWidgets.QHBoxLayout(self)

        # left: file lists
        left = QtWidgets.QVBoxLayout()
        left.addWidget(QtWidgets.QLabel("<b>Recordings</b> (data/)"))
        self.lstRec = QtWidgets.QListWidget()
        self.lstRec.currentItemChanged.connect(lambda it, _: it and self.open_recording(it.data(QtCore.Qt.UserRole)))
        left.addWidget(self.lstRec, 3)
        left.addWidget(QtWidgets.QLabel("<b>Calibrations</b> (calibration_reports/)"))
        self.lstCal = QtWidgets.QListWidget()
        self.lstCal.currentItemChanged.connect(lambda it, _: it and self.open_calibration(it.data(QtCore.Qt.UserRole)))
        left.addWidget(self.lstCal, 2)
        h = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Refresh")
        b.clicked.connect(self.refresh_lists)
        h.addWidget(b)
        b = QtWidgets.QPushButton("Open file…")
        b.clicked.connect(self.open_any)
        h.addWidget(b)
        left.addLayout(h)
        wl = QtWidgets.QWidget()
        wl.setLayout(left)
        wl.setMinimumWidth(400)
        wl.setMaximumWidth(460)
        lay.addWidget(wl, 0)

        # right: one page per file type
        self.stack = QtWidgets.QStackedWidget()
        lay.addWidget(self.stack, 1)
        empty = QtWidgets.QLabel("Pick a recording or a calibration on the left.\n"
                                 "Right-click a plot → Export… to save it as PNG / SVG / CSV.")
        empty.setAlignment(QtCore.Qt.AlignCenter)
        self.stack.addWidget(empty)
        self.stack.addWidget(self._build_rec_page())
        self.stack.addWidget(self._build_cal_page())
        self.refresh_lists()

    # --- pages ---
    def _build_rec_page(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.lblRec = QtWidgets.QLabel()
        self.lblRec.setWordWrap(True)
        self.lblRec.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        v.addWidget(self.lblRec)
        h = QtWidgets.QHBoxLayout()
        self.chkRaw = QtWidgets.QCheckBox("zeroed (raw)")
        self.chkRaw.setChecked(True)
        self.chkCalV = QtWidgets.QCheckBox("calibrated")
        self.chkCalV.setChecked(True)
        self.chkAbs = QtWidgets.QCheckBox("absolute (+ zero)")
        self.chkAbs.setToolTip("Adds the zero stored on the board at recording time (from the _meta.json).")
        self.chkSel = QtWidgets.QCheckBox("select a time window")
        self.chkSel.setToolTip("Drag the shaded region; the statistics below follow it.")
        for c in (self.chkRaw, self.chkCalV, self.chkAbs, self.chkSel):
            c.toggled.connect(self.draw_recording)
            h.addWidget(c)
        h.addStretch(1)
        v.addLayout(h)
        self.recPlots, self.regions = {}, {}
        for i in SENSORS:
            p = pg.PlotWidget()
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("left", f"S{i}", units="pF")
            p.setLabel("bottom", "time", units="s")
            p.addLegend()
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
            r = pg.LinearRegionItem()
            r.sigRegionChanged.connect(self._region_moved)
            self.recPlots[i], self.regions[i] = p, r
            v.addWidget(p, 1)
        self.recPlots[2].setXLink(self.recPlots[1])
        self.tblRec = QtWidgets.QTableWidget(0, 7)
        self.tblRec.setHorizontalHeaderLabels(["series", "samples", "mean pF", "sd fF", "min pF", "max pF", "p-p fF"])
        self.tblRec.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tblRec.setMaximumHeight(130)
        self.tblRec.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        v.addWidget(self.tblRec)
        return w

    def _build_cal_page(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.lblCal = QtWidgets.QLabel()
        self.lblCal.setWordWrap(True)
        self.lblCal.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        v.addWidget(self.lblCal)
        h = QtWidgets.QHBoxLayout()
        self.pCalFit = pg.PlotWidget(title="measured → true (fit curve)")
        self.pCalFit.setLabel("bottom", "measured (zeroed) pF")
        self.pCalFit.setLabel("left", "true pF")
        self.pCalFit.showGrid(x=True, y=True, alpha=0.3)
        self.pCalFit.addLegend()
        self.pCalErr = pg.PlotWidget(title="error after calibration (corrected − true)")
        self.pCalErr.setLabel("bottom", "true pF")
        self.pCalErr.setLabel("left", "error pF")
        self.pCalErr.showGrid(x=True, y=True, alpha=0.3)
        self.pCalErr.getAxis("left").enableAutoSIPrefix(False)
        self.pCalErr.addLegend()
        h.addWidget(self.pCalFit, 1)
        h.addWidget(self.pCalErr, 1)
        v.addLayout(h, 3)
        self.tblCal = QtWidgets.QTableWidget(0, 6)
        self.tblCal.setHorizontalHeaderLabels(["kind", "true pF", "measured pF", "noise fF", "corrected pF", "error pF"])
        self.tblCal.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tblCal.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        v.addWidget(self.tblCal, 2)
        return w

    # --- files ---
    def refresh_lists(self):
        def fill(lst, paths):
            lst.blockSignals(True)
            lst.clear()
            for p in paths:
                it = QtWidgets.QListWidgetItem(p.name)
                it.setToolTip(str(p))
                it.setData(QtCore.Qt.UserRole, str(p))
                lst.addItem(it)
            lst.blockSignals(False)
        recs = sorted((p for p in OUT_DIR.glob("*.csv")), reverse=True) if OUT_DIR.exists() else []
        cals = []
        if REPORT_DIR.exists():
            cals = sorted(REPORT_DIR.glob("profile_*.json")) + \
                sorted(REPORT_DIR.glob("*_calibration_*.json"), reverse=True)
        fill(self.lstRec, recs)
        fill(self.lstCal, cals)

    def open_any(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open recording or calibration", str(HERE.parent),
                                                        "Recordings / calibrations (*.csv *.json)")
        if not path:
            return
        if path.endswith("_meta.json"):
            path = path.replace("_meta.json", ".csv")
        if path.endswith(".csv") and "_calibration_" in os.path.basename(path):
            path = path[:-4] + ".json"                  # calibration CSV -> its JSON report
        (self.open_recording if path.endswith(".csv") else self.open_calibration)(path)

    # --- recordings ---
    def open_recording(self, path):
        try:
            cols, meta = load_recording(path)
            t = np.asarray(cols["Time_s"], float)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Recording", f"Could not read {path}:\n{e}")
            return
        self.rec = (t - t[0] if len(t) else t, cols, meta or {}, path)
        cfg = (meta or {}).get("board_config", {})
        n, dur = len(t), (t[-1] - t[0]) if len(t) > 1 else 0
        info = [f"<b>{os.path.basename(path)}</b>", f"{n} samples over {dur:.1f} s"
                + (f" ({n / dur:.0f}/s)" if dur else "")]
        if meta:
            info.append(f"recorded {meta.get('created', '?')} via {meta.get('port', '?')}")
        for i in SENSORS:
            if cfg:
                st = cols.get(f"S{i}_status", [])
                bad = sum(1 for x in st if x not in ("ok", "off"))
                info.append(f"S{i}: {cfg.get(f'mode{i}', '?')} on {cfg.get(f'pins{i}', '?')}, zero "
                            f"{cfg.get(f'zero{i}_pf', 0):.3f} pF, " + ("calibrated" if cfg.get(f"cal{i}") else "no calibration")
                            + (f", {bad} non-ok samples (gaps)" if bad else ""))
        if not cfg:
            info.append("no _meta.json next to the file: board settings unknown")
        self.lblRec.setText("<br>".join(info))
        self.chkAbs.setEnabled(bool(cfg))
        t0 = self.rec[0]
        for i in SENSORS:
            if len(t0) > 1:
                self.regions[i].blockSignals(True)
                self.regions[i].setRegion((t0[0] + 0.25 * dur, t0[0] + 0.75 * dur))
                self.regions[i].blockSignals(False)
        self.stack.setCurrentIndex(1)
        self.draw_recording()
        for p in self.recPlots.values():
            p.enableAutoRange()

    def _series(self):
        t, cols, meta, _ = self.rec
        cfg = meta.get("board_config", {})
        out = {}
        for i, name in zip(SENSORS, ("C12", "C34")):
            add = cfg.get(f"zero{i}_pf", 0.0) if self.chkAbs.isChecked() and self.chkAbs.isEnabled() else 0.0
            raw, cal = cols.get(f"{name}_raw_pF"), cols.get(f"{name}_cal_pF")
            s = []
            if self.chkRaw.isChecked() and isinstance(raw, np.ndarray) and np.isfinite(raw).any():
                s.append(("absolute" if add else "zeroed", raw + add, COLORS[i]))
            if self.chkCalV.isChecked() and isinstance(cal, np.ndarray) and np.isfinite(cal).any():
                s.append(("calibrated", cal, "k"))
            out[i] = s
        return t, out

    def draw_recording(self):
        if not self.rec:
            return
        t, series = self._series()
        for i in SENSORS:
            p = self.recPlots[i]
            p.clear()
            if p.plotItem.legend:
                p.plotItem.legend.clear()
            for name, y, col in series[i]:
                pen = pg.mkPen(col, width=1.2, style=QtCore.Qt.DashLine if name == "calibrated" else QtCore.Qt.SolidLine)
                p.plot(t, y, pen=pen, name=name, connect="finite")
            any_data = any(isinstance(self.rec[1].get(f"{n}_{k}_pF"), np.ndarray) and
                           np.isfinite(self.rec[1][f"{n}_{k}_pF"]).any()
                           for n in (("C12",) if i == 1 else ("C34",)) for k in ("raw", "cal"))
            p.setVisible(any_data)
            if self.chkSel.isChecked():
                p.addItem(self.regions[i])
        self._fill_stats()

    def _region_moved(self, r):
        lo, hi = r.getRegion()
        for o in self.regions.values():
            if o is not r:
                o.blockSignals(True)
                o.setRegion((lo, hi))
                o.blockSignals(False)
        self._fill_stats()

    def _fill_stats(self):
        t, series = self._series()
        mask = np.ones(len(t), bool)
        label = "all"
        if self.chkSel.isChecked() and len(t):
            lo, hi = self.regions[1].getRegion()
            mask = (t >= lo) & (t <= hi)
            label = f"{lo:.1f}–{hi:.1f} s"
        rows = []
        for i in SENSORS:
            for name, y, _ in series[i]:
                st = _stats(y[mask])
                if st:
                    rows.append([f"S{i} {name} ({label})", str(st["n"]), f"{st['mean']:.4f}", f"{st['sd'] * 1000:.1f}",
                                 f"{st['lo']:.4f}", f"{st['hi']:.4f}", f"{(st['hi'] - st['lo']) * 1000:.1f}"])
        self.tblRec.setRowCount(len(rows))
        for r, vals in enumerate(rows):
            for c, val in enumerate(vals):
                self.tblRec.setItem(r, c, QtWidgets.QTableWidgetItem(val))

    # --- calibrations ---
    def open_calibration(self, path):
        try:
            d = json.load(open(path))
        except (OSError, ValueError) as e:
            QtWidgets.QMessageBox.warning(self, "Calibration", f"Could not read {path}:\n{e}")
            return
        if "channels" in d:
            self._show_profile(path, d)
        elif "fit" in d:
            self._show_report(path, d)
        else:
            QtWidgets.QMessageBox.warning(self, "Calibration", "Not a calibration report or profile.")
            return
        self.stack.setCurrentIndex(2)

    def _show_profile(self, path, d):
        """Profiles only have coefficients: show them and plot the points from each source report."""
        lines = [f"<b>{os.path.basename(path)}</b> · board {d.get('board_usb_serial', '?')}",
                 f"<i>{d.get('equation', '')}</i>"]
        reports = []
        for name, ch in d.get("channels", {}).items():
            lines.append(f"<b>{name}</b>: {ch.get('mode', '?')} on {ch.get('pins', '?')} · gain {ch['gain']:.6g}, "
                         f"shift {ch['shift_pf']:+.6g} pF, quad {ch.get('quad', 0):.6g} · {ch.get('points', '?')} points, "
                         f"max residual {ch.get('max_residual_pf', float('nan')):.3f} pF · "
                         + ("valid for measured {:.3f} … {:.3f} pF · ".format(*ch["calibrated_range_measured_pf"])
                            if ch.get("calibrated_range_measured_pf") else "") +
                         f"{ch.get('calibrated', '')} · from {ch.get('source_report', '—')}")
            src = Path(path).parent / ch.get("source_report", "")
            if ch.get("source_report") and src.exists():
                try:
                    reports.append(json.load(open(src)))
                except ValueError:
                    pass
        self.lblCal.setText("<br>".join(lines))
        self._plot_cal(reports[0] if len(reports) == 1 else None, d)

    def _show_report(self, path, d):
        f = d["fit"]
        ref = [p for p in d.get("points", []) if p.get("kind") != "verify"]
        ms = [p["measured_pf"] for p in ref]
        lines = [f"<b>{os.path.basename(path)}</b> · {d.get('created', '')} · board {d.get('board_usb_serial', '?')} "
                 f"· firmware {d.get('firmware', '?')}",
                 f"Sensor {d.get('sensor', '?')}: {d.get('pins_label', d.get('mode', '?'))} · zero "
                 f"{d.get('zero_pf') if d.get('zero_pf') is not None else float('nan'):.4f} pF · "
                 f"{d.get('measure_secs', '?')} s per point at {d.get('rate', '?')} S/s",
                 f"fit ({'quadratic' if f.get('degree') == 2 else 'linear'}): <b>true = {f.get('quad', 0):.6g}·m² "
                 f"+ {f['gain']:.6g}·m {f['shift_pf']:+.6g} pF</b> · {f.get('points', '?')} points · max residual "
                 f"<b>{f.get('max_residual_pf', float('nan')):.3f} pF</b>"
                 + (f" · valid for measured {min(ms):.3f} … {max(ms):.3f} pF" if ms else "")]
        self.lblCal.setText("<br>".join(lines))
        self._plot_cal(d, None)

    def _plot_cal(self, rep, profile):
        self.pCalFit.clear()
        self.pCalErr.clear()
        for p in (self.pCalFit, self.pCalErr):
            if p.plotItem.legend:
                p.plotItem.legend.clear()
        rows = []
        curves = []                       # (label, a0, a1, a2, m range)
        if rep:
            f = rep["fit"]
            pts = rep.get("points", [])
            ms = [p["measured_pf"] for p in pts if p.get("kind") != "verify"] or [0, 1]
            curves.append(("fit", f["shift_pf"], f["gain"], f.get("quad", 0.0), (min(ms), max(ms))))
            sym = {"reference": ("o", COLORS[1]), "baseline": ("s", "#2e7d32"), "verify": ("t", COLORS[2])}
            for kind, (sb, col) in sym.items():
                k = [p for p in pts if p.get("kind") == kind]
                if not k:
                    continue
                m = np.array([p["measured_pf"] for p in k])
                tr = np.array([p["true_pf"] for p in k])
                cor = np.array([p.get("corrected_pf", np.nan) if p.get("corrected_pf") is not None else np.nan for p in k])
                self.pCalFit.plot(m, tr, pen=None, symbol=sb, symbolBrush=col, symbolSize=9, name=f"{kind} ({len(k)})")
                self.pCalErr.plot(tr, cor - tr, pen=None, symbol=sb, symbolBrush=col, symbolSize=9, name=kind)
                for p, c in zip(k, cor):
                    rows.append([kind, p["true_pf"], p["measured_pf"], p.get("noise_ff"), c,
                                 c - p["true_pf"] if np.isfinite(c) else None])
        elif profile:
            for name, ch in profile.get("channels", {}).items():
                rng = ch.get("calibrated_range_measured_pf") or [0, 10]
                curves.append((name, ch["shift_pf"], ch["gain"], ch.get("quad", 0.0), tuple(rng)))
        for k, (label, a0, a1, a2, (lo, hi)) in enumerate(curves):
            span = max(hi - lo, 1e-6)
            mm = np.linspace(lo - 0.1 * span, hi + 0.1 * span, 200)
            self.pCalFit.plot(mm, a2 * mm * mm + a1 * mm + a0, pen=pg.mkPen("k" if k == 0 else COLORS[2], width=1.5),
                              name=f"{label} curve")
        self.pCalErr.addItem(pg.InfiniteLine(0, angle=0, pen=pg.mkPen("#9e9e9e", style=QtCore.Qt.DashLine)))
        fmt = lambda x, n: "" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{n}f}"
        self.tblCal.setRowCount(len(rows))
        for r, (kind, tr, m, nz, c, e) in enumerate(rows):
            vals = [kind, fmt(tr, 3), fmt(m, 4), fmt(nz, 1), fmt(c, 4), fmt(e, 4)]
            for col, val in enumerate(vals):
                it = QtWidgets.QTableWidgetItem(val)
                if col == 5 and e is not None and math.isfinite(e) and abs(e) > 0.5:
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                self.tblCal.setItem(r, col, it)


# --- main window ---
class Studio(QtWidgets.QMainWindow):
    def __init__(self, simulate=False):
        super().__init__()
        self.setWindowTitle("FDC Studio · FDC1004 board (firmware v3)")
        self.resize(1400, 860)
        self.q = queue.Queue()
        self.link = None
        self.fw3 = False
        self.board_cfg = {}
        self.ring = Ring()
        self.ms0 = None
        self.last_ms = None
        self.rec = None
        self.pending_probe = None
        self.latest = {i: (None, "?", None) for i in SENSORS}
        self.rate_win = collections.deque(maxlen=400)
        self.settings = QtCore.QSettings("FDCStudio", "fdc_studio")

        pg.setConfigOptions(antialias=False, background="w", foreground="k")
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central)
        h.addWidget(self._build_side(), 0)
        self.tabs = QtWidgets.QTabWidget()
        h.addWidget(self.tabs, 1)
        self.tabs.addTab(self._build_live(), "Live")
        self.wizard = Wizard(self)
        self.tabs.addTab(self.wizard, "Calibration wizard")
        self.viewer = Viewer(self)
        self.tabs.addTab(self.viewer, "Viewer")
        self.tabs.currentChanged.connect(lambda k: self.tabs.widget(k) is self.viewer and self.viewer.refresh_lists())
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.tabs.addTab(self.log, "Log")

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)
        if simulate:
            self.connect_sim()

    # --- UI ---
    def _build_side(self):
        w = QtWidgets.QWidget()
        w.setFixedWidth(330)
        v = QtWidgets.QVBoxLayout(w)

        g = QtWidgets.QGroupBox("Connection")
        gv = QtWidgets.QGridLayout(g)
        self.cmbPort = QtWidgets.QComboBox()
        b = QtWidgets.QPushButton("↻")
        b.setFixedWidth(30)
        b.clicked.connect(self.refresh_ports)
        gv.addWidget(self.cmbPort, 0, 0, 1, 2)
        gv.addWidget(b, 0, 2)
        self.btnUsb = QtWidgets.QPushButton("Connect USB")
        self.btnUsb.clicked.connect(self.connect_usb)
        gv.addWidget(self.btnUsb, 1, 0, 1, 3)
        self.edHost = QtWidgets.QLineEdit(self.settings.value("host", ""))
        self.edHost.setPlaceholderText("board IP for WiFi")
        b = QtWidgets.QPushButton("WiFi")
        b.clicked.connect(self.connect_wifi)
        gv.addWidget(self.edHost, 2, 0, 1, 2)
        gv.addWidget(b, 2, 2)
        b = QtWidgets.QPushButton("Simulator")
        b.clicked.connect(self.connect_sim)
        gv.addWidget(b, 3, 0)
        b = QtWidgets.QPushButton("Disconnect")
        b.clicked.connect(self.disconnect)
        gv.addWidget(b, 3, 1, 1, 2)
        self.lblConn = QtWidgets.QLabel("not connected")
        self.lblConn.setWordWrap(True)
        gv.addWidget(self.lblConn, 4, 0, 1, 3)
        v.addWidget(g)
        self.refresh_ports()

        self.sensorUi = {}
        for i in SENSORS:
            g = QtWidgets.QGroupBox(f"Sensor {i}  (Sensor{i}A / Sensor{i}B)")
            gl = QtWidgets.QGridLayout(g)
            chk = QtWidgets.QCheckBox("measure")
            chk.setChecked(True)
            chk.clicked.connect(self.send_meas)
            mode = QtWidgets.QComboBox()
            mode.addItem("diff  (A − B, ±16 pF raw)", "diff")
            mode.addItem("se  (A to GND, auto-range)", "se")
            mode.activated.connect(lambda _=0, i=i: self.send_mode(i))
            bz = QtWidgets.QPushButton("Zero")
            bz.setToolTip("Nothing connected to A/B (cables in place) for absolute values; "
                          "sensor at rest to measure changes from rest. Stored on the board.")
            bz.clicked.connect(lambda _=0, i=i: self.zero(i))
            bp = QtWidgets.QPushButton("Probe")
            bp.clicked.connect(lambda _=0, i=i: self.send(f"probe {i}"))
            info = QtWidgets.QLabel("")
            info.setWordWrap(True)
            info.setStyleSheet("color:#555")
            info.setMinimumHeight(34)
            gl.addWidget(chk, 0, 0)
            gl.addWidget(mode, 0, 1, 1, 2)
            gl.addWidget(bz, 1, 1)
            gl.addWidget(bp, 1, 2)
            gl.addWidget(info, 2, 0, 1, 3)
            if True:                                   # simulator only
                sim = QtWidgets.QDoubleSpinBox()
                sim.setRange(0, 120)
                sim.setSuffix(" pF")
                sim.setDecimals(2)
                sim.setPrefix("Virtual capacitor ")
                sim.valueChanged.connect(lambda val, i=i: self._sim_cap(i, val))
                sim.setVisible(False)
                gl.addWidget(sim, 3, 0, 1, 3)
            self.sensorUi[i] = dict(chk=chk, mode=mode, info=info, sim=sim)
            v.addWidget(g)

        g = QtWidgets.QGroupBox("Converter")
        gl = QtWidgets.QGridLayout(g)
        self.cmbRate = QtWidgets.QComboBox()
        self.cmbRate.addItems(["100", "200", "400"])
        self.cmbRate.setCurrentText("400")
        self.spnAvg = QtWidgets.QSpinBox()
        self.spnAvg.setRange(1, 64)
        self.spnStream = QtWidgets.QSpinBox()
        self.spnStream.setRange(0, 5000)
        self.spnStream.setValue(10)
        self.spnStream.setSuffix(" ms")
        self.chkSettle = QtWidgets.QCheckBox("settle (drop 1st conversion after switching sensor)")
        gl.addWidget(QtWidgets.QLabel("rate S/s"), 0, 0)
        gl.addWidget(self.cmbRate, 0, 1)
        gl.addWidget(QtWidgets.QLabel("average"), 1, 0)
        gl.addWidget(self.spnAvg, 1, 1)
        gl.addWidget(QtWidgets.QLabel("stream"), 2, 0)
        gl.addWidget(self.spnStream, 2, 1)
        gl.addWidget(self.chkSettle, 3, 0, 1, 2)
        b = QtWidgets.QPushButton("Apply")
        b.clicked.connect(self.apply_conv)
        gl.addWidget(b, 4, 0)
        b = QtWidgets.QPushButton("Scan pins")
        b.clicked.connect(lambda: self.send("scan"))
        gl.addWidget(b, 4, 1)
        b = QtWidgets.QPushButton("Status")
        b.clicked.connect(lambda: self.send("status"))
        gl.addWidget(b, 5, 0)
        b = QtWidgets.QPushButton("pins default")
        b.clicked.connect(lambda: self.send("pins default"))
        gl.addWidget(b, 5, 1)
        v.addWidget(g)

        self.edCmd = QtWidgets.QLineEdit()
        self.edCmd.setPlaceholderText("command (help, probe 1, reg, ...) + Enter")
        self.edCmd.returnPressed.connect(self._cmd_line)
        v.addWidget(self.edCmd)
        self.lblMsg = QtWidgets.QLabel("")
        self.lblMsg.setWordWrap(True)
        self.lblMsg.setStyleSheet("background:#f6f6f6;padding:4px")
        self.lblMsg.setMinimumHeight(90)
        self.lblMsg.setAlignment(QtCore.Qt.AlignTop)
        v.addWidget(self.lblMsg)
        v.addStretch(1)
        return w

    def _build_live(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        cards = QtWidgets.QHBoxLayout()
        self.card = {}
        for i in SENSORS:
            f = QtWidgets.QFrame()
            f.setFrameShape(QtWidgets.QFrame.StyledPanel)
            fl = QtWidgets.QVBoxLayout(f)
            t = QtWidgets.QLabel(f"<b style='color:{COLORS[i]}'>Sensor {i}</b>")
            val = QtWidgets.QLabel("—")
            val.setStyleSheet("font-size:20px;font-weight:600")
            st = QtWidgets.QLabel("-")
            st.setStyleSheet("color:white;background:#757575;padding:2px 6px;border-radius:3px")
            st.setMaximumHeight(26)
            fl.addWidget(t)
            fl.addWidget(val)
            fl.addWidget(st)
            cards.addWidget(f)
            self.card[i] = dict(val=val, st=st)
        side = QtWidgets.QVBoxLayout()
        self.lblRate = QtWidgets.QLabel("0 lines/s")
        self.spnWin = QtWidgets.QDoubleSpinBox()
        self.spnWin.setRange(1, 600)
        self.spnWin.setValue(20)
        self.spnWin.setPrefix("window ")
        self.spnWin.setSuffix(" s")
        self.chkCal = QtWidgets.QCheckBox("show calibrated")
        self.chkCal.setChecked(True)
        self.chkOverlay = QtWidgets.QCheckBox("one plot")
        self.chkOverlay.toggled.connect(self._layout_plots)
        self.btnRec = QtWidgets.QPushButton("● Record CSV")
        self.btnRec.setCheckable(True)
        self.btnRec.toggled.connect(self.toggle_rec)
        self.edName = QtWidgets.QLineEdit("capacitance")
        b = QtWidgets.QPushButton("Clear plot")
        b.clicked.connect(self.ring.clear)
        for x in (self.lblRate, self.spnWin, self.chkCal, self.chkOverlay, self.edName, self.btnRec, b):
            side.addWidget(x)
        cards.addLayout(side)
        v.addLayout(cards)

        self.plots = {}
        self.curves = {}
        self.plotBox = QtWidgets.QVBoxLayout()
        v.addLayout(self.plotBox, 1)
        for i in SENSORS:
            p = pg.PlotWidget()
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("left", f"S{i}", units="pF")
            p.setLabel("bottom", "time", units="s")
            p.setClipToView(True)
            p.setDownsampling(auto=True, mode="peak")
            self.plots[i] = p
            self.curves[i] = (p.plot(pen=pg.mkPen(COLORS[i], width=1.2), name="zeroed"),
                              p.plot(pen=pg.mkPen("k", width=1.2, style=QtCore.Qt.DashLine), name="calibrated"))
        self.plotAll = pg.PlotWidget()
        self.plotAll.showGrid(x=True, y=True, alpha=0.3)
        self.plotAll.addLegend()
        self.curvesAll = {i: self.plotAll.plot(pen=pg.mkPen(COLORS[i], width=1.2), name=f"S{i}") for i in SENSORS}
        self._layout_plots()
        return w

    def _layout_plots(self):
        for wdg in list(self.plots.values()) + [self.plotAll]:
            wdg.setParent(None)
        if self.chkOverlay.isChecked():
            self.plotBox.addWidget(self.plotAll)
        else:
            for p in self.plots.values():
                self.plotBox.addWidget(p)

    # --- connection ---
    def refresh_ports(self):
        self.cmbPort.clear()
        if serial is None:
            return
        for p in serial.tools.list_ports.comports():
            self.cmbPort.addItem(f"{p.device}  {p.description}", p.device)

    def _start(self, link):
        self.disconnect()
        self.q = queue.Queue()
        link.q = self.q
        self.link = link
        link.start()
        self.ms0 = None
        self.ring.clear()
        sim = isinstance(link, SimLink)
        for i in SENSORS:
            self.sensorUi[i]["sim"].setVisible(sim)
        self.lblConn.setText(f"connected: {link.name}")
        self.send("get")

    def connect_usb(self):
        port = self.cmbPort.currentData()
        if not port:
            return
        try:
            self._start(SerialLink(port, self.q))
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "USB", f"{e}\n\nClose the Arduino Serial Monitor / other apps.")

    def connect_wifi(self):
        host = self.edHost.text().strip()
        if not host:
            return
        try:
            self._start(TcpLink(host, self.q))
            self.settings.setValue("host", host)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "WiFi", str(e))

    def connect_sim(self):
        self._start(SimLink(self.q))

    def disconnect(self):
        if self.rec:
            self.btnRec.setChecked(False)
        if self.link:
            self.link.close()
        self.link = None
        self.lblConn.setText("not connected")

    def _sim_cap(self, i, val):
        if isinstance(self.link, SimLink):
            self.link.cm[i] = val

    # --- commands ---
    def send(self, text):
        if not self.link:
            return
        self.say("> " + ("wifipass ****" if text.lower().startswith("wifipass") else text))
        self.link.send(text)

    def _cmd_line(self):
        t = self.edCmd.text().strip()
        if t:
            self.send(t)
            self.edCmd.clear()

    def send_meas(self):
        a, b = (self.sensorUi[i]["chk"].isChecked() for i in SENSORS)
        if not (a or b):
            self.sensorUi[1]["chk"].setChecked(True)
            a = True
        self.send("meas " + ("all" if a and b else "1" if a else "2"))

    def send_mode(self, i):
        if not self.fw3:
            QtWidgets.QMessageBox.information(self, "Mode", "Measurement modes need firmware v3.")
            return
        m = self.sensorUi[i]["mode"].currentData()
        if m != self.board_cfg.get(f"mode{i}"):
            r = QtWidgets.QMessageBox.question(self, "Change mode",
                                               f"Switch sensor {i} to {m}? Its zero and calibration are cleared.")
            if r == QtWidgets.QMessageBox.Yes:
                self.send(f"mode {i} {m}")
            else:
                self._sync_ui()

    def zero(self, i):
        r = QtWidgets.QMessageBox.question(
            self, f"Zero sensor {i}",
            "Absolute values: disconnect the sensor from A and B (leave the cables), then Yes.\n"
            "Changes from rest: leave the sensor connected and untouched, then Yes.\n\n"
            "The zero is stored on the board (not redone at power-up).")
        if r == QtWidgets.QMessageBox.Yes:
            self.send(f"zero {i}")

    def apply_conv(self):
        self.send(f"rate {self.cmbRate.currentText()}")
        self.send(f"avg {self.spnAvg.value()}")
        self.send(f"stream {self.spnStream.value()}")
        if self.fw3:
            self.send(f"settle {1 if self.chkSettle.isChecked() else 0}")

    # --- incoming data ---
    def say(self, text):
        self.log.appendPlainText(f"{datetime.datetime.now():%H:%M:%S}  {text}")

    def _tick(self):
        end = time.perf_counter() + 0.02
        while time.perf_counter() < end:
            try:
                d = self.q.get_nowait()
            except queue.Empty:
                break
            self._ingest(d)
        self._redraw()

    def _ingest(self, d):
        ev = d.get("event")
        if ev in ("msg", "text"):
            t = d.get("text", "")
            self.say(t)
            if ev == "msg":
                self.lblMsg.setText(t)
        elif ev == "config":
            self.board_cfg = d
            self.fw3 = bool(d.get("fw3"))
            self._sync_ui()
        elif ev == "probe":
            self.say("probe: " + json.dumps(d))
            if self.pending_probe:
                wiz, which = self.pending_probe
                self.pending_probe = None
                wiz.on_probe(which, d)
        elif ev == "scan":
            self.say("scan: " + json.dumps(d))
        elif ev == "disconnected":
            self.say("connection lost: " + d.get("text", ""))
            self.disconnect()
        elif "c1_pf" in d or "c2_pf" in d:
            self._sample(d)

    def _sample(self, d):
        ms = d.get("millis")
        if ms is not None:
            if self.ms0 is None or (self.last_ms is not None and ms < self.last_ms):
                self.ms0 = ms
            self.last_ms = ms
            t = (ms - self.ms0) / 1000.0
        else:
            t = time.time()
        vals, cals, sts, cds = {}, {}, {}, {}
        for i in SENSORS:
            x = d.get(f"c{i}_pf")
            c = d.get(f"c{i}_cal_pf")
            st = d.get(f"st{i}")
            if st is None:                          # v2 firmware: infer the rail
                st = "off" if x is None else ("sat" if abs(x) >= 15.9 else "ok")
            ok = st == "ok"
            vals[i] = float(x) if x is not None and ok else float("nan")
            cals[i] = float(c) if c is not None and ok else float("nan")
            sts[i], cds[i] = st, d.get(f"cd{i}", "")
            self.latest[i] = (x, st, c)
        self.ring.append(t, vals, cals)
        self.rate_win.append(time.time())
        if self.rec:
            self.rec.write(t, ms, vals, cals, sts, cds)
        self.wizard.feed({"v": {i: d.get(f"c{i}_pf") for i in SENSORS},
                          "c": {i: d.get(f"c{i}_cal_pf") for i in SENSORS}, "st": sts})

    def _sync_ui(self):
        d = self.board_cfg
        meas = int(d.get("meas", 3))
        for i in SENSORS:
            u = self.sensorUi[i]
            u["chk"].setChecked(bool(meas & (1 << (i - 1))))
            m = d.get(f"mode{i}", "diff")
            u["mode"].setCurrentIndex(0 if m == "diff" else 1)
            u["mode"].setEnabled(self.fw3)
            pins = d.get(f"pins{i}", "?")
            z = d.get(f"zeroed{i}")
            zt = "" if z is None else (f"zero {d.get(f'zero{i}_pf', 0):.3f} pF" if z else "<b style='color:#c62828'>NOT ZEROED</b>")
            ct = "calibrated" if d.get(f"cal{i}") else "no calibration"
            cd = f", CAPDAC {d.get(f'capdac{i}')}" if m == "se" else ""
            u["info"].setText(f"pins {pins}{cd} · {zt} · {ct}")
        self.cmbRate.setCurrentText(str(d.get("rate", 400)))
        self.spnAvg.setValue(int(d.get("avg", 1)))
        self.spnStream.setValue(int(d.get("stream_ms", 10)))
        self.chkSettle.setChecked(bool(d.get("settle", 0)))
        self.chkSettle.setEnabled(self.fw3)
        fw = f"firmware v{d.get('ver', '?')}" if self.fw3 else "firmware v2 (update to v3 for modes, status, probe)"
        warn = " · <b style='color:#c62828'>sensors share a pin</b>" if d.get("shared") else ""
        errs = f" · I2C errors {d.get('i2c_err')}, resets {d.get('resets')}" if self.fw3 else ""
        if self.link:
            self.lblConn.setText(f"connected: {self.link.name}<br>{fw}{errs}{warn}")

    def _redraw(self):
        now = time.time()
        while self.rate_win and now - self.rate_win[0] > 2:
            self.rate_win.popleft()
        self.lblRate.setText(f"{len(self.rate_win) / 2:.0f} lines/s")
        for i in SENSORS:
            x, st, c = self.latest[i]
            card = self.card[i]
            if x is None:
                card["val"].setText("—")
            else:
                s = f"{x:+.4f} pF"
                if c is not None and self.chkCal.isChecked():
                    s += f"   →  <b>{c:.4f} pF</b> cal"
                card["val"].setText(s)
            card["st"].setText(ST_TEXT.get(st, st))
            card["st"].setStyleSheet(f"color:white;background:{ST_COLOR.get(st, '#757575')};padding:2px 6px;border-radius:3px")
        data = self.ring.last(self.spnWin.value())
        if data is None:
            return
        t, v, c = data
        for i in SENSORS:
            if self.chkOverlay.isChecked():
                y = c[i] if self.chkCal.isChecked() and np.isfinite(c[i]).any() else v[i]
                self.curvesAll[i].setData(t, y, connect="finite")
            else:
                self.curves[i][0].setData(t, v[i], connect="finite")
                if self.chkCal.isChecked() and np.isfinite(c[i]).any():
                    self.curves[i][1].setData(t, c[i], connect="finite")
                else:
                    self.curves[i][1].setData([], [])

    # --- recording ---
    def toggle_rec(self, on):
        if on:
            OUT_DIR.mkdir(parents=True, exist_ok=True)
            name = "".join(ch for ch in self.edName.text() if ch.isalnum() or ch in "-_") or "capacitance"
            path = OUT_DIR / f"{datetime.datetime.now():%Y%m%d_%H%M%S}_{name}.csv"
            meta = {"created": now_iso(), "port": self.link.name if self.link else "", "board_config": self.board_cfg,
                    "note": "C12/C34_raw_pF = zeroed board reading (cN_pf); _cal_pF = board calibration (cN_cal_pf). "
                            "Samples whose status is not ok are NaN."}
            self.rec = Recorder(path, meta)
            self.btnRec.setText("■ Stop recording")
            self.say(f"recording {path}")
        else:
            if self.rec:
                self.rec.close()
                self.say(f"saved {self.rec.path} ({self.rec.n} lines)")
            self.rec = None
            self.btnRec.setText("● Record CSV")

    def closeEvent(self, e):
        self.disconnect()
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--simulate", action="store_true")
    args = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    w = Studio(simulate=args.simulate)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
