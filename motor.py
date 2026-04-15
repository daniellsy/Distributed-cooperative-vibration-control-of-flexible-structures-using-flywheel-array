#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import re
import time
import argparse
import threading
from collections import deque
from typing import Optional
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# Optional high-performance plotting
try:
    import pyqtgraph as pg
    HAS_PYQTGRAPH = True
except Exception:
    pg = None  # type: ignore
    HAS_PYQTGRAPH = False
# Try to import pyserial but keep feature optional (we'll show clear message if missing)
try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except Exception:
    serial = None
    HAS_PYSERIAL = False

# Try to import PyQt5 but keep feature optional
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                                 QPushButton, QComboBox, QLabel, QStatusBar, QLineEdit, QTabWidget, QGridLayout, QCheckBox, QFileDialog)
    from PyQt5.QtCore import QTimer, QThread, pyqtSignal
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    HAS_PYQT = True
except ImportError:
    HAS_PYQT = False
    # Fallback placeholders to satisfy static analyzers when PyQt is missing
    QApplication = None  # type: ignore
    QMainWindow = object  # type: ignore
    QWidget = object  # type: ignore
    QVBoxLayout = object  # type: ignore
    QHBoxLayout = object  # type: ignore
    QGridLayout = object  # type: ignore
    QPushButton = object  # type: ignore
    QComboBox = object  # type: ignore
    QLabel = object  # type: ignore
    QStatusBar = object  # type: ignore
    QLineEdit = object  # type: ignore
    QCheckBox = object  # type: ignore
    class QFileDialog:  # type: ignore
        @staticmethod
        def getSaveFileName(*args, **kwargs):
            return ('', '')
    QTimer = object  # type: ignore
    QThread = object  # type: ignore
    def pyqtSignal(*args, **kwargs):  # type: ignore
        return None
    FigureCanvas = object  # type: ignore
    class Figure:  # type: ignore
        pass

# 正则：捕获 Target/Actual/gyroZ（浮点数或整数，可带正负）以及可选的 target1/del1/del2
# 支持如下两种格式：
#   Target = 39.42 Actual = 39.42 gyroZ= 0.00
#   Target = 39.42 Actual = 39.42 gyroZ= 0.00 target1= 39.42 del1= 0.00,del2= 0.00
pattern  = re.compile(
    r"Target\s*=\s*([-+]?\d+(?:\.\d+)?)\s+"
    r"Actual\s*=\s*([-+]?\d+(?:\.\d+)?)\s+"
    r"gyroZ\s*=\s*([-+]?\d+(?:\.\d+)?)",
    re.IGNORECASE
)

# Precompiled individual patterns for optional fields (order/commas not required)
_opt_target1 = re.compile(r"\btarget1\s*=\s*([-+]?\d+(?:\.\d+)?)", re.IGNORECASE)
_opt_del1    = re.compile(r"\bdel1\s*=\s*([-+]?\d+(?:\.\d+)?)", re.IGNORECASE)
_opt_del2    = re.compile(r"\bdel2\s*=\s*([-+]?\d+(?:\.\d+)?)", re.IGNORECASE)


def extract_Target23_from_line(line):
    """从一行文本中提取 Target/Actual/gyroZ，找不到返回 None。
    兼容扩展格式（行尾可有 target1/del1/del2），但仅返回前三个值。
    """
    m = pattern.search(line)
    if not m:
        return None
    try:
        return float(m.group(1)), float(m.group(2)), float(m.group(3))
    except Exception:
        return None


def extract_extended_from_line(line):
    """从一行文本中提取最多 6 个值：Target, Actual, gyroZ, target1, del1, del2。
    仅前三个为必选，后三个若不存在则返回 None。支持任意空格/无逗号/顺序。
    匹配失败返回 None。
    """
    m = pattern.search(line)
    if not m:
        return None
    try:
        t = float(m.group(1))
        a = float(m.group(2))
        g = float(m.group(3))
    except Exception:
        return None

    # Optional fields: search independently to accept arbitrary spacing/order
    def _parse_opt(rx):
        m2 = rx.search(line)
        if m2:
            try:
                return float(m2.group(1))
            except Exception:
                return None
        return None

    t1 = _parse_opt(_opt_target1)
    d1 = _parse_opt(_opt_del1)
    d2 = _parse_opt(_opt_del2)
    return t, a, g, t1, d1, d2


def extract_Target23(txt_file):
    """返回 DataFrame：['Target', 'Actual', 'gyroZ', 'target1', 'del1', 'del2'] 从指定文本文件中解析所有行。
    兼容旧格式（仅前三列）；对缺失的扩展列填充 NaN。
    """
    Target, Actual, gyroZ = [], [], []
    target1_list, del1_list, del2_list = [], [], []
    with open(txt_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            res = extract_extended_from_line(line)
            if res:
                a, b, c, t1, d1, d2 = res
                Target.append(a)
                Actual.append(b)
                gyroZ.append(c)
                target1_list.append(t1)
                del1_list.append(d1)
                del2_list.append(d2)
    return pd.DataFrame({
        'Target': Target,
        'Actual': Actual,
        'gyroZ': gyroZ,
        'target1': target1_list,
        'del1': del1_list,
        'del2': del2_list,
    })


class SerialReader(threading.Thread):
    """线程：从串口读取文本行，解析并推入共享缓冲区（三个 deque）。"""

    def __init__(self, port, baud, queues, lock, stop_event, encoding='utf-8', timeout=1.0):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.queues = queues  # dict with keys 'Target','Actual','gyroZ'
        self.lock = lock
        self.stop_event = stop_event
        self.encoding = encoding
        self.timeout = timeout
        self.ser = None

    def run(self):
        if not HAS_PYSERIAL:
            print('❌ pyserial 未安装，无法打开串口。请运行: pip install pyserial')
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print(f'✅ 已打开串口 {self.port} @ {self.baud}')
        except Exception as e:
            print(f'❌ 无法打开串口 {self.port}: {e}')
            return

        try:
            # 逐行读取文本（设备应发送以换行结束的 ASCII/UTF-8 文本）
            while not self.stop_event.is_set():
                try:
                    raw = self.ser.readline()
                except Exception as e:
                    print('串口读取错误:', e)
                    break
                if not raw:
                    continue
                try:
                    line = raw.decode(self.encoding, errors='ignore')
                except Exception:
                    line = str(raw)
                res = extract_Target23_from_line(line)
                if res:
                    a, b, c = res
                    with self.lock:
                        self.queues['Target'].append(a)
                        self.queues['Actual'].append(b)
                        self.queues['gyroZ'].append(c)
        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass


class FileSimulator(threading.Thread):
    """线程：从文件中按速率读取行并推入共享队列，便于在无串口硬件时进行回放/测试。"""

    def __init__(self, txt_file, queues, lock, stop_event, interval_ms=50, repeat=False):
        super().__init__(daemon=True)
        self.txt_file = txt_file
        self.queues = queues
        self.lock = lock
        self.stop_event = stop_event
        self.interval_ms = interval_ms
        self.repeat = repeat

    def run(self):
        if not os.path.exists(self.txt_file):
            print(f'❌ 模拟文件不存在: {self.txt_file}')
            return
        try:
            while not self.stop_event.is_set():
                with open(self.txt_file, 'r', encoding='utf-8', errors='ignore') as f:
                    for line in f:
                        if self.stop_event.is_set():
                            return
                        res = extract_Target23_from_line(line)
                        if res:
                            a, b, c = res
                            with self.lock:
                                self.queues['Target'].append(a)
                                self.queues['Actual'].append(b)
                                self.queues['gyroZ'].append(c)
                        time.sleep(self.interval_ms / 1000.0)
                if not self.repeat:
                    break
        except Exception as e:
            print('文件模拟器发生错误:', e)


def run_realtime_plot(queues, lock, window_len=500, interval=50):
    """使用 matplotlib 实时绘图（类似示波器）。
    queues: dict of deques for 'Target','Actual','gyroZ'
    window_len: 可视窗口的点数
    interval: 刷新间隔（毫秒）
    """
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))
    x = list(range(-window_len + 1, 1))
    line1, = ax.plot(x, [0] * window_len, label='Target', alpha=0.9)
    line2, = ax.plot(x, [0] * window_len, label='Actual', alpha=0.9)
    line3, = ax.plot(x, [0] * window_len, label='gyroZ', alpha=0.9)

    ax.set_xlim(-window_len + 1, 0)
    ax.set_xlabel('样本（最新在右侧）')
    ax.set_ylabel('角速度 (deg/s)')
    ax.set_title('实时 Target / Actual / gyroZ (示波器模式)')
    ax.grid(True)
    ax.legend()

    # Initial data
    data1 = deque([0.0] * window_len, maxlen=window_len)
    data2 = deque([0.0] * window_len, maxlen=window_len)
    data3 = deque([0.0] * window_len, maxlen=window_len)

    try:
        while True:
            # drain available samples from shared queues
            with lock:
                while queues['Target']:
                    data1.append(queues['Target'].popleft())
                while queues['Actual']:
                    data2.append(queues['Actual'].popleft())
                while queues['gyroZ']:
                    data3.append(queues['gyroZ'].popleft())

            # update line data
            line1.set_ydata(list(data1))
            line2.set_ydata(list(data2))
            line3.set_ydata(list(data3))

            # autoscale y a bit
            all_y = list(data1) + list(data2) + list(data3)
            if all_y:
                ymin = min(all_y)
                ymax = max(all_y)
                if ymin == ymax:
                    ymin -= 0.5
                    ymax += 0.5
                ax.set_ylim(ymin - abs(0.1 * ymin), ymax + abs(0.1 * ymax))

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(interval / 1000.0)

    except KeyboardInterrupt:
        print('\n退出实时绘图（KeyboardInterrupt）')
    except Exception as e:
        print('绘图出现错误:', e)
    finally:
        plt.close(fig)


class SerialReaderThread(QThread):
    """Reads from serial port in a separate thread and emits data."""
    # 扩展信号：6 个 float（若缺失以 NaN 代替）
    newData = pyqtSignal(float, float, float, float, float, float)  # type: ignore
    error = pyqtSignal(str)  # type: ignore

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self._is_running = True
        self.ser = None
        self.send_queue = deque()

    def run(self):
        try:
            # 设置较短的读写超时，避免阻塞导致 GUI 卡死
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.2)
        except Exception as e:
            self.error.emit(str(e))  # type: ignore
            return

        while self._is_running:
            # 1. Handle sending from queue
            try:
                if self.send_queue:
                    data_to_send = self.send_queue.popleft()
                    if isinstance(data_to_send, str):
                        b = data_to_send.encode('utf-8')
                    else:
                        b = data_to_send
                    if not b.endswith(b'\n'):
                        b = b + b'\n'
                    # 仅调用 write，避免 flush 的潜在阻塞
                    self.ser.write(b)
            except getattr(serial, 'SerialTimeoutException', Exception) as e:  # type: ignore[attr-defined]
                if self._is_running:
                    self.error.emit(f'Send timeout: {e}')  # type: ignore
            except Exception as e:
                if self._is_running:
                    self.error.emit(f'Send error: {e}')  # type: ignore

            # 2. Handle reading
            try:
                # readline() will block for timeout duration if no data
                line = self.ser.readline().decode('utf-8', errors='ignore')
                if line:
                    # 解析扩展字段
                    res = extract_extended_from_line(line)
                    if res:
                        t, a, g, t1, d1, d2 = res
                        # 将缺失值以 NaN 传出
                        t1 = t1 if t1 is not None else math.nan
                        d1 = d1 if d1 is not None else math.nan
                        d2 = d2 if d2 is not None else math.nan
                        self.newData.emit(t, a, g, t1, d1, d2)  # type: ignore
            except serial.SerialException:
                if self._is_running:
                    self.error.emit('Serial read error.')  # type: ignore
                break  # Exit loop on major error
            except Exception:
                # Ignore other minor read errors (e.g. decoding)
                pass

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        # QThread will auto-emit its built-in 'finished' signal when run() returns

    def stop(self):
        self._is_running = False

    def send(self, data):
        """
        Queue a command to be sent by the worker thread.
        This method is thread-safe and non-blocking. Returns True when queued, False otherwise.
        """
        try:
            self.send_queue.append(data)
            return True
        except Exception:
            return False


class PlotCanvas(FigureCanvas):
    """Matplotlib canvas for the plot.

    Performance-oriented: supports downsampling and optional fixed y-limits
    to avoid expensive autoscaling when plotting multiple panels.
    """

    def __init__(self, parent=None, width=5, height=4, dpi=100, window_len=500, fixed_ylim=None):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

        self.window_len = window_len
        self.fixed_ylim = fixed_ylim
        self.x_data = list(range(-window_len + 1, 1))
        # 扩展绘图：新增 target1, del1, del2 三条曲线
        self.lines = {
            'Target': self.axes.plot(self.x_data, [0] * window_len, label='Target', alpha=0.9)[0],
            'Actual': self.axes.plot(self.x_data, [0] * window_len, label='Actual', alpha=0.9)[0],
            'gyroZ': self.axes.plot(self.x_data, [0] * window_len, label='gyroZ', alpha=0.9)[0],
            'target1': self.axes.plot(self.x_data, [0] * window_len, label='target1', alpha=0.7, linestyle='--')[0],
            'del1': self.axes.plot(self.x_data, [0] * window_len, label='del1', alpha=0.7, linestyle=':')[0],
            'del2': self.axes.plot(self.x_data, [0] * window_len, label='del2', alpha=0.7, linestyle='-.')[0],
        }
        self.axes.legend(loc='upper right')
        self.axes.grid(True)
        self.axes.set_xlabel('Samples (most recent on the right)')
        self.axes.set_ylabel('Value')
        self.axes.set_title('Real-time Target / Actual / gyroZ / target1 / del1 / del2')

        # If fixed y-limits requested, set them and skip autoscaling during updates
        if self.fixed_ylim is not None:
            try:
                ymin, ymax = self.fixed_ylim
                self.axes.set_ylim(ymin, ymax)
            except Exception:
                pass

    def update_figure(self, data):
        # Downsample each series to at most _max_points to avoid drawing huge arrays
        max_points = getattr(self, '_max_points', 800)
        autoscale_every = getattr(self, '_autoscale_every', 50)
        frame = getattr(self, '_frame', 0) + 1
        self._frame = frame

        def downsample(seq, limit):
            # seq is a deque or iterable; convert to list once
            y = list(seq)
            n = len(y)
            if n <= limit:
                x = list(range(-n + 1, 1))
                return x, y
            step = max(1, n // limit)
            y2 = y[::step]
            x2 = list(range(-len(y2) + 1, 1))
            return x2, y2

        xT, yT = downsample(data['Target'], max_points)
        xA, yA = downsample(data['Actual'], max_points)
        xG, yG = downsample(data['gyroZ'], max_points)
        x1, y1 = downsample(data['target1'], max_points)
        xD1, yD1 = downsample(data['del1'], max_points)
        xD2, yD2 = downsample(data['del2'], max_points)

        # Use set_data(x, y) to keep x/y consistent after downsampling
        try:
            self.lines['Target'].set_data(xT, yT)
            self.lines['Actual'].set_data(xA, yA)
            self.lines['gyroZ'].set_data(xG, yG)
            self.lines['target1'].set_data(x1, y1)
            self.lines['del1'].set_data(xD1, yD1)
            self.lines['del2'].set_data(xD2, yD2)
        except Exception:
            # Fallback to set_ydata if set_data not available for the artist
            self.lines['Target'].set_ydata(yT)
            self.lines['Actual'].set_ydata(yA)
            self.lines['gyroZ'].set_ydata(yG)
            self.lines['target1'].set_ydata(y1)
            self.lines['del1'].set_ydata(yD1)
            self.lines['del2'].set_ydata(yD2)

        # Only autoscale if no fixed_ylim was specified
        if self.fixed_ylim is None and frame % autoscale_every == 0:
            all_y = []
            for arr in (yT, yA, yG, y1, yD1, yD2):
                for v in arr:
                    if v is not None and not (isinstance(v, float) and math.isnan(v)):
                        all_y.append(v)
            if all_y:
                ymin, ymax = min(all_y), max(all_y)
                if ymin == ymax:
                    ymin -= 0.5
                    ymax += 0.5
                padding = (ymax - ymin) * 0.1
                self.axes.set_ylim(ymin - padding, ymax + padding)

        # Use draw_idle to coalesce multiple updates
        self.draw_idle()


class PlotCanvasPG(QWidget):
    """PyQtGraph-based canvas for fast real-time plotting.

    Draws 6 curves: Target, Actual, gyroZ, target1, del1, del2.
    Uses fixed Y range if provided to avoid autoscale cost.
    """

    def __init__(self, parent=None, width=6, height=3, dpi=100, window_len=500, fixed_ylim=None):
        super().__init__(parent)
        self.window_len = window_len
        self.fixed_ylim = fixed_ylim

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.plot = pg.PlotWidget()
        layout.addWidget(self.plot)

        # Visuals
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel('bottom', 'Samples (most recent on the right)')
        self.plot.setLabel('left', 'Value')
        self.plot.setTitle('Real-time Target / Actual / gyroZ / target1 / del1 / del2')
        if self.fixed_ylim is not None:
            try:
                ymin, ymax = self.fixed_ylim
                # Some versions of pyqtgraph may not accept 'padding' kw arg; use positional only
                self.plot.setYRange(ymin, ymax)
            except Exception:
                pass

        # Legend
        try:
            self.plot.addLegend(offset=(10, 10))
        except Exception:
            pass

        # Create curves
        mkPen = pg.mkPen  # type: ignore[attr-defined]
        self.curves = {
            'Target':  self.plot.plot(pen=mkPen((220, 20, 60), width=2), name='Target'),
            'Actual':  self.plot.plot(pen=mkPen((34, 139, 34), width=2), name='Actual'),
            'gyroZ':   self.plot.plot(pen=mkPen((30, 144, 255), width=2), name='gyroZ'),
            'target1': self.plot.plot(pen=mkPen((148, 0, 211), width=1.5), name='target1'),
            'del1':    self.plot.plot(pen=mkPen((128, 128, 128), width=1.5), name='del1'),
            'del2':    self.plot.plot(pen=mkPen((255, 140, 0), width=1.5), name='del2'),
        }

        # Attempt to enable clip/downsampling hints to reduce work
        for c in self.curves.values():
            try:
                c.setClipToView(True)
            except Exception:
                pass
            try:
                c.setDownsampling(auto=True, method='peak')
            except Exception:
                pass

    def update_figure(self, data):
        """Update all curves using full-length X (implicit indices) and let
        pyqtgraph handle Y downsampling automatically during rendering.
        This preserves your given X data points while reducing draw cost.
        """
        try:
            yT = np.asarray(list(data['Target']), dtype=float)
            yA = np.asarray(list(data['Actual']), dtype=float)
            yG = np.asarray(list(data['gyroZ']), dtype=float)
            y1 = np.asarray(list(data['target1']), dtype=float)
            yD1 = np.asarray(list(data['del1']), dtype=float)
            yD2 = np.asarray(list(data['del2']), dtype=float)

            # Provide full-length arrays; autoDownsample keeps X implicit and unchanged
            self.curves['Target'].setData(y=yT, autoDownsample=True)
            self.curves['Actual'].setData(y=yA, autoDownsample=True)
            self.curves['gyroZ'].setData(y=yG, autoDownsample=True)
            self.curves['target1'].setData(y=y1, autoDownsample=True)
            self.curves['del1'].setData(y=yD1, autoDownsample=True)
            self.curves['del2'].setData(y=yD2, autoDownsample=True)
        except Exception:
            # Be permissive; ignore transient issues
            pass


class SerialMonitorWidget(QWidget):
    """A self-contained serial monitor panel with controls, plot, status label and send command UI."""

    def __init__(self, window_len=500, refresh_ms=50, title: Optional[str] = None):
        super().__init__()
        self.window_len = window_len
        self.refresh_ms = refresh_ms
        self.title = title or ''

        # Data buffers：扩展存储六个字段
        self.data = {
            'Target': deque([0.0] * window_len, maxlen=window_len),
            'Actual': deque([0.0] * window_len, maxlen=window_len),
            'gyroZ': deque([0.0] * window_len, maxlen=window_len),
            'target1': deque([math.nan] * window_len, maxlen=window_len),
            'del1': deque([math.nan] * window_len, maxlen=window_len),
            'del2': deque([math.nan] * window_len, maxlen=window_len),
        }
        self.serial_thread: Optional[SerialReaderThread] = None

        # Layouts and widgets
        root = QVBoxLayout(self)
        header = QHBoxLayout()
        if self.title:
            header.addWidget(QLabel(self.title))
        self.port_combo = QComboBox()
        self.refresh_button = QPushButton('Refresh')
        self.connect_button = QPushButton('Connect')
        self.disconnect_button = QPushButton('Disconnect')
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baud_combo.setCurrentText('115200')

        header.addWidget(QLabel('Port:'))
        header.addWidget(self.port_combo)
        header.addWidget(self.refresh_button)
        header.addWidget(QLabel('Baud:'))
        header.addWidget(self.baud_combo)
        header.addWidget(self.connect_button)
        header.addWidget(self.disconnect_button)
        header.addStretch()
        root.addLayout(header)

        # Plot canvas: use PyQtGraph for higher performance
        self.canvas = PlotCanvasPG(self, width=6, height=3, dpi=100, window_len=self.window_len, fixed_ylim=(-40, 50))
        root.addWidget(self.canvas)

        # Command send controls
        send_layout = QHBoxLayout()
        send_layout.addWidget(QLabel('Cmd:'))
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText('输入要发送的命令 (会自动追加换行)')
        send_layout.addWidget(self.cmd_input)
        self.send_button = QPushButton('Send')
        send_layout.addWidget(self.send_button)

        # 快捷按钮（五个）：WIT_start / 电机启动 / 协同控制 / 分散控制 / 停止
        self.wit_start_button = QPushButton('WIT_start')   # 111
        self.motor_start_button = QPushButton('电机启动')    # 222
        self.coop_button = QPushButton('协同控制')          # 333
        self.decentral_button = QPushButton('分散控制')      # 444
        self.stop_button = QPushButton('停止')              # 555
        send_layout.addWidget(self.wit_start_button)
        send_layout.addWidget(self.motor_start_button)
        send_layout.addWidget(self.coop_button)
        send_layout.addWidget(self.decentral_button)
        send_layout.addWidget(self.stop_button)

        send_layout.addStretch()
        root.addLayout(send_layout)

        # Per-panel status label and update checkbox + Save button
        footer = QHBoxLayout()
        self.status_label = QLabel('Idle')
        footer.addWidget(self.status_label)
        self.update_checkbox = QCheckBox('Update')
        self.update_checkbox.setChecked(True)
        self.update_checkbox.toggled.connect(self.set_active)  # type: ignore
        footer.addWidget(self.update_checkbox)
        # Save button
        self.save_button = QPushButton('Save')
        footer.addWidget(self.save_button)
        footer.addStretch()
        root.addLayout(footer)

        # Connections
        self.refresh_button.clicked.connect(self.refresh_ports)  # type: ignore
        self.connect_button.clicked.connect(self.connect_serial)  # type: ignore
        self.disconnect_button.clicked.connect(self.disconnect_serial)  # type: ignore
        self.send_button.clicked.connect(self.on_send_clicked)  # type: ignore
        self.save_button.clicked.connect(self.save_to_excel)  # type: ignore

        # 快捷按钮事件（映射）：111=WIT_start, 222=电机启动, 333=协同控制, 444=分散控制, 555=停止
        self.wit_start_button.clicked.connect(lambda: self._send_quick_cmd('111'))  # type: ignore
        self.motor_start_button.clicked.connect(lambda: self._send_quick_cmd('222'))  # type: ignore
        self.coop_button.clicked.connect(lambda: self._send_quick_cmd('333'))  # type: ignore
        self.decentral_button.clicked.connect(lambda: self._send_quick_cmd('444'))  # type: ignore
        self.stop_button.clicked.connect(lambda: self._send_quick_cmd('555'))  # type: ignore

        # Initially disable send controls
        self.send_button.setEnabled(False)
        self.cmd_input.setEnabled(False)
        self.wit_start_button.setEnabled(False)
        self.motor_start_button.setEnabled(False)
        self.coop_button.setEnabled(False)
        self.decentral_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        # Timer for throttled redraws
        self.timer = QTimer(self)  # type: ignore
        self.timer.setInterval(self.refresh_ms)  # type: ignore
        self.timer.timeout.connect(self.on_timer_update)  # type: ignore
        # start timer by default; can be toggled via the Update checkbox
        self.timer.start()  # type: ignore

    def set_active(self, active: bool):
        """Enable/disable this panel's periodic redraws (does not stop serial thread).
        When unchecked, the panel will continue to collect data but will not redraw.
        """
        try:
            if active:
                self.timer.start()
            else:
                self.timer.stop()
        except Exception:
            # Best-effort: ignore errors
            pass

    def refresh_ports(self):
        self.port_combo.clear()
        if not HAS_PYSERIAL:
            self.status_label.setText('pyserial not installed')
            return
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.status_label.setText('No ports found')
        else:
            self.port_combo.addItems([p.device for p in ports])
            self.status_label.setText(f'Found {len(ports)} ports')

    def connect_serial(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        if not port:
            self.status_label.setText('Select a port first')
            return
        if self.serial_thread is not None:
            self.status_label.setText('Already connected')
            return
        self.serial_thread = SerialReaderThread(port, baud)
        self.serial_thread.newData.connect(self.buffer_data)  # type: ignore
        self.serial_thread.error.connect(lambda m: self.status_label.setText(f'Error: {m}'))  # type: ignore
        self.serial_thread.finished.connect(self.on_serial_finished)  # type: ignore
        self.serial_thread.start()  # type: ignore

        # Track current port for saving filename
        try:
            self.current_port = port
        except Exception:
            self.current_port = None  # type: ignore

        # enable send UI and 快捷按钮
        self.send_button.setEnabled(True)
        self.cmd_input.setEnabled(True)
        self.wit_start_button.setEnabled(True)
        self.motor_start_button.setEnabled(True)
        self.coop_button.setEnabled(True)
        self.decentral_button.setEnabled(True)
        self.stop_button.setEnabled(True)

        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)
        self.status_label.setText(f'Connecting to {port} @ {baud}...')

    def disconnect_serial(self):
        if self.serial_thread:
            self.serial_thread.stop()
        self.on_serial_finished()

    def on_serial_finished(self):
        # disable send UI and 快捷按钮
        self.send_button.setEnabled(False)
        self.cmd_input.setEnabled(False)
        self.wit_start_button.setEnabled(False)
        self.motor_start_button.setEnabled(False)
        self.coop_button.setEnabled(False)
        self.decentral_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
        if self.serial_thread:
            self.status_label.setText('Disconnected')
            self.serial_thread = None
        # clear current port info
        try:
            self.current_port = None
        except Exception:
            pass

    def buffer_data(self, Target, Actual, gyroZ, target1, del1, del2):
        self.data['Target'].append(Target)
        self.data['Actual'].append(Actual)
        self.data['gyroZ'].append(gyroZ)
        self.data['target1'].append(target1)
        self.data['del1'].append(del1)
        self.data['del2'].append(del2)

    def on_timer_update(self):
        self.canvas.update_figure(self.data)

    def on_send_clicked(self):
        cmd = self.cmd_input.text()
        if not cmd:
            return
        if not self.serial_thread:
            self.status_label.setText('Not connected')
            return
        try:
            ok = self.serial_thread.send(cmd)
            if ok:
                self.status_label.setText(f'Sent: {cmd}')
            else:
                # serial_thread will also emit error signal which updates status_label
                self.status_label.setText('Send failed')
        except Exception as e:
            self.status_label.setText(f'Send error: {e}')

    def _send_quick_cmd(self, cmd_str: str):
        """内部使用：发送快捷命令并更新状态标签。"""
        if not self.serial_thread:
            self.status_label.setText('Not connected')
            return
        try:
            ok = self.serial_thread.send(cmd_str)
            if ok:
                self.status_label.setText(f'Sent: {cmd_str}')
            else:
                self.status_label.setText('Send failed')
        except Exception as e:
            self.status_label.setText(f'Send error: {e}')

    def close(self):  # pylint: disable=arguments-differ
        # Ensure background thread is stopped when panel is closed
        try:
            if self.serial_thread:
                self.serial_thread.stop()
        except Exception:
            pass
        self.serial_thread = None
        super().close()

    def save_to_excel(self):
        """Save current window data to ./result/<port>_<timestamp>.xlsx automatically (no dialog).
        Fallback to CSV if Excel writer engine is unavailable.
        """
        try:
            # Snapshot data
            tgt = list(self.data['Target'])
            act = list(self.data['Actual'])
            gyr = list(self.data['gyroZ'])
            t1  = list(self.data['target1'])
            d1  = list(self.data['del1'])
            d2  = list(self.data['del2'])
            n = max(len(tgt), len(act), len(gyr), len(t1), len(d1), len(d2))
            if n == 0:
                self.status_label.setText('No data to save')
                return
            def pad(lst, size):
                return lst + [math.nan] * (size - len(lst))
            df = pd.DataFrame({
                'index': list(range(n)),
                'Target': pad(tgt, n),
                'Actual': pad(act, n),
                'gyroZ': pad(gyr, n),
                'target1': pad(t1, n),
                'del1': pad(d1, n),
                'del2': pad(d2, n),
            })

            # Determine base directory: alongside this script
            base_dir = os.path.dirname(os.path.abspath(__file__))
            result_dir = os.path.join(base_dir, 'result')
            os.makedirs(result_dir, exist_ok=True)

            # Determine port name for filename
            port_name = None
            try:
                port_name = getattr(self, 'current_port', None) or self.port_combo.currentText()
            except Exception:
                port_name = None
            if not port_name:
                port_name = 'unknown'

            # Sanitize port for filename
            safe = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in port_name)
            ts = time.strftime('%Y%m%d_%H%M%S')
            xlsx_path = os.path.join(result_dir, f'{safe}_{ts}.xlsx')

            # Try Excel first; fallback to CSV on failure
            try:
                df.to_excel(xlsx_path, index=False)
                self.status_label.setText(f'Saved: result/{os.path.basename(xlsx_path)}')
            except Exception:
                csv_path = os.path.join(result_dir, f'{safe}_{ts}.csv')
                df.to_csv(csv_path, index=False, encoding='utf-8-sig')
                self.status_label.setText(f'Excel engine missing, saved CSV: result/{os.path.basename(csv_path)}')
        except Exception as e:
            try:
                self.status_label.setText(f'Save failed: {e}')
            except Exception:
                pass


class PyQtSerialPlotter(QMainWindow):
    """Main window hosting three SerialMonitorWidget panels side by side."""

    def __init__(self, window_len=500, refresh_ms=50):
        super().__init__()
        self.setWindowTitle('Real-time Serial Plotter (3 Panels)')
        self.setGeometry(100, 100, 1500, 700)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QGridLayout(central)

        # Create three panels
        self.panel1 = SerialMonitorWidget(window_len=window_len, refresh_ms=refresh_ms, title='Channel 1')
        self.panel2 = SerialMonitorWidget(window_len=window_len, refresh_ms=refresh_ms, title='Channel 2')
        self.panel3 = SerialMonitorWidget(window_len=window_len, refresh_ms=refresh_ms, title='Channel 3')
        self.panel4 = SerialMonitorWidget(window_len=window_len, refresh_ms=refresh_ms, title='Channel 4')

        layout.addWidget(self.panel1, 0, 0)
        layout.addWidget(self.panel2, 0, 1)
        layout.addWidget(self.panel3, 1, 0)
        layout.addWidget(self.panel4, 1, 1)

        # Optional: status bar for general messages
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage('Ready')

    def closeEvent(self, event):
        # Ensure child panels stop their threads
        for p in (self.panel1, self.panel2, self.panel3, self.panel4):
            try:
                p.close()
            except Exception:
                pass
        event.accept()


# 入口

def main():
    parser = argparse.ArgumentParser(description='实时读取 Target/Actual/gyroZ 并绘图（串口或文件）')
    parser.add_argument('--mode', choices=['serial', 'file', 'parse', 'sim', 'gui'], default='gui',
                        help='运行模式: serial=从串口实时读取, file=从文本文件读取并绘图, parse=仅解析并打印摘要, sim=从文本回放以测试实时绘图, gui=启动图形用户界面')
    parser.add_argument('--port', default='COM8', help='串口设备 (仅 mode=serial)')
    parser.add_argument('--baud', type=int, default=115200, help='波特率 (仅 mode=serial)')
    parser.add_argument('--window', type=int, default=3000, help='实时显示窗口长度（点数）')
    parser.add_argument('--interval', type=int, default=100, help='刷新间隔（毫秒）')
    parser.add_argument('--file', help='要解析或绘图的文件（mode=file 或 mode=parse 或 mode=sim）')
    parser.add_argument('--dir', help='要解析并合并的目录（如果未指定 --file）')
    parser.add_argument('--repeat', action='store_true', help='对于 sim 模式，是否重复回放文件')
    args = parser.parse_args()

    if args.mode == 'gui':
        if not HAS_PYQT:
            print("❌ PyQt5 is not installed. Please run: pip install PyQt5")
            return
        if not HAS_PYSERIAL:
            print("❌ pyserial is not installed. Please run: pip install pyserial")
            # GUI can still run to show the interface, but will not be able to connect.
        app = QApplication([])
        main_window = PyQtSerialPlotter(window_len=args.window, refresh_ms=args.interval)
        main_window.show()
        app.exec_()
        return

    if args.mode == 'parse':
        # 仅解析并打印摘要（用于测试或 CI）
        if not args.file:
            print('请使用 --file 指定要解析的文本文件')
            return
        df = extract_Target23(args.file)
        print(f'解析完成: {args.file} -> {len(df)} 行匹配到 Target/Actual/gyroZ')
        if not df.empty:
            print(df.head().to_string(index=False))
        return

    if args.mode == 'file':
        # 支持单个文件或者目录（合并目录下所有 .txt）
        files = []
        if args.file:
            files = [args.file]
        else:
            # if dir specified use it, else current dir
            d = args.dir or '.'
            files = [os.path.join(d, f) for f in os.listdir(d) if f.endswith('.txt')]
        if not files:
            print('❌ 找不到 .txt 文件可供解析')
            return
        df_all = pd.concat([extract_Target23(f) for f in files], ignore_index=True)
        if df_all.empty:
            print('❌ 未匹配到 Target/Actual/gyroZ 数据')
            return

        plt.figure(figsize=(15, 8))
        plt.plot(df_all.index, df_all['Target'], label='Target', alpha=0.8)
        plt.plot(df_all.index, df_all['Actual'], label='Actual', alpha=0.8)
        plt.plot(df_all.index, df_all['gyroZ'], label='gyroZ', alpha=0.8)
        plt.title('Target / Actual / gyroZ 时间序列')
        plt.xlabel('样本序号')
        plt.ylabel('角速度 (deg/s)')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()
        return

    if args.mode == 'sim':
        # 模拟回放文件到实时绘图队列（便于无硬件时测试）
        if not args.file:
            print('请使用 --file 指定要回放的文本文件 (mode=sim)')
            return
        queues = {
            'Target': deque(maxlen=10000),
            'Actual': deque(maxlen=10000),
            'gyroZ': deque(maxlen=10000),
        }
        lock = threading.Lock()
        stop_event = threading.Event()

        sim = FileSimulator(args.file, queues, lock, stop_event, interval_ms=args.interval, repeat=args.repeat)
        sim.start()
        try:
            run_realtime_plot(queues, lock, window_len=args.window, interval=args.interval)
        finally:
            stop_event.set()
            sim.join(timeout=1.0)
        return

    # serial mode: prepare shared buffers and start reader thread
    if args.mode == 'serial':
        if not HAS_PYSERIAL:
            print('❌ pyserial 未安装。安装: pip install pyserial')
            return
        queues = {
            'Target': deque(maxlen=10000),
            'Actual': deque(maxlen=10000),
            'gyroZ': deque(maxlen=10000),
        }
        lock = threading.Lock()
        stop_event = threading.Event()

        reader = SerialReader(args.port, args.baud, queues, lock, stop_event)
        reader.start()

        try:
            run_realtime_plot(queues, lock, window_len=args.window, interval=args.interval)
        finally:
            stop_event.set()
            reader.join(timeout=1.0)


if __name__ == '__main__':
    main()
