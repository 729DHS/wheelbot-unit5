#!/usr/bin/env python3
"""
数字孪生可视化 — 双面板五连杆实时显示。

通过 UDP 接收 serial_bridge.py 转发的电机角度，正运动学解算
左右腿连杆姿态，PyQt6 QPainter 并排渲染。

用法:
  # 先启动串口桥接
  python3 tools/serial_bridge.py --port /dev/ttyACM0

  # 再启动可视化 (UDP 默认 127.0.0.1:9999)
  python3 tools/twin_display.py

  # 或离线回放 CSV
  python3 tools/twin_display.py --csv feedback.csv

依赖: pip install PyQt6
"""

import argparse
import math
import re
import socket
import sys
import threading
import time
from dataclasses import dataclass

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from PyQt6.QtWidgets import QApplication, QHBoxLayout, QLabel, QVBoxLayout, QWidget

# ---------------------------------------------------------------------------
# 运动学常量 (与 linkage_kinematics.h 一致)
# ---------------------------------------------------------------------------
L1_MM = 107.4  # 第一等效连杆长
L2_MM = 128.0  # 第二等效连杆长
WORKSPACE_MIN = 20.6
WORKSPACE_MAX = 235.4

# M1/M2 → 左腿, M3/M4 → 右腿
# 角度偏置 (robot cali 零点, 后续可通过 UI 调整)
LEFT_OFFSET_A = math.radians(-162.4)
LEFT_OFFSET_B = math.radians(-10.0)
RIGHT_OFFSET_A = math.radians(-162.4)
RIGHT_OFFSET_B = math.radians(-10.0)

# 显示缩放 (像素/mm)
SCALE = 2.5
PANEL_W = 600
PANEL_H = 700
ORIGIN_X = PANEL_W // 2
ORIGIN_Y = PANEL_H - 80


# ---------------------------------------------------------------------------
# 运动学
# ---------------------------------------------------------------------------
@dataclass
class Pose:
    x_mm: float
    y_mm: float


@dataclass
class JointAngles:
    theta_a: float
    theta_b: float


def lk_forward(angles: JointAngles) -> Pose:
    """正运动学: 关节角 → 末端位置"""
    x = L1_MM * math.cos(angles.theta_a) + L2_MM * math.cos(angles.theta_b)
    y = L1_MM * math.sin(angles.theta_a) + L2_MM * math.sin(angles.theta_b)
    return Pose(x, y)


def to_pixel(mm_x: float, mm_y: float) -> QPointF:
    """机构坐标 (原点在左上方, +X 右 +Y 上) → 像素坐标 (+Y 下)"""
    px = ORIGIN_X + mm_x * SCALE
    py = ORIGIN_Y - mm_y * SCALE
    return QPointF(px, py)


# ---------------------------------------------------------------------------
# 电机 → 关节角
# ---------------------------------------------------------------------------
def motor_to_theta(motor_rad: float, offset_rad: float) -> float:
    """电机角度 → 机构帧关节角 (符号取决于实际安装方向)"""
    return motor_rad + offset_rad


# ---------------------------------------------------------------------------
# UDP 接收线程
# ---------------------------------------------------------------------------
# 匹配 CSV 行 (兼容 5 列旧格式和 12 列新格式)
CSV_RE = re.compile(
    r"(\d+),"                    # t_ms
    r"(-?\d+\.\d+),"             # m1
    r"(-?\d+\.\d+),"             # m2
    r"(-?\d+\.\d+),"             # m3
    r"(-?\d+\.\d+)"              # m4
)


class UdpReceiver(threading.Thread):
    """后台线程: 接收 UDP CSV 帧, 更新最新电机角度"""

    def __init__(self, host: str, port: int):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.lock = threading.Lock()
        self.t_ms: int = 0
        self.m1: float = 0.0
        self.m2: float = 0.0
        self.m3: float = 0.0
        self.m4: float = 0.0
        self.frame_count: int = 0
        self.running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.host, self.port))
        except OSError as e:
            print(f"UDP bind 失败 {self.host}:{self.port}: {e}")
            return

        sock.settimeout(0.5)
        while self.running:
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            text = data.decode("utf-8", errors="replace")
            m = CSV_RE.search(text)
            if not m:
                continue

            with self.lock:
                self.t_ms = int(m.group(1))
                self.m1 = float(m.group(2))
                self.m2 = float(m.group(3))
                self.m3 = float(m.group(4))
                self.m4 = float(m.group(5))
                self.frame_count += 1

        sock.close()

    def get_latest(self):
        with self.lock:
            return (self.t_ms, self.m1, self.m2, self.m3, self.m4, self.frame_count)


# ---------------------------------------------------------------------------
# 离线 CSV 播放器
# ---------------------------------------------------------------------------
class CsvPlayer:
    """逐帧读取 CSV 文件, 模拟实时数据源"""

    def __init__(self, path: str):
        self.frames: list[tuple[int, float, float, float, float]] = []
        with open(path) as f:
            for line in f:
                m = CSV_RE.search(line)
                if m:
                    self.frames.append(
                        (
                            int(m.group(1)),
                            float(m.group(2)),
                            float(m.group(3)),
                            float(m.group(4)),
                            float(m.group(5)),
                        )
                    )
        self.idx = 0
        self.start_time = time.time()
        self.first_t_ms = self.frames[0][0] if self.frames else 0

    def get_next(self) -> tuple[int, float, float, float, float, int] | None:
        if not self.frames or self.idx >= len(self.frames):
            return None
        elapsed_ms = (time.time() - self.start_time) * 1000
        target_t = self.first_t_ms + elapsed_ms
        while self.idx < len(self.frames) and self.frames[self.idx][0] <= target_t:
            self.idx += 1
        if self.idx == 0:
            self.idx = 1
        t_ms, a, b, c, d = self.frames[self.idx - 1]
        return (t_ms, a, b, c, d, self.idx)


# ---------------------------------------------------------------------------
# 单腿绘制面板
# ---------------------------------------------------------------------------
class LegPanel(QWidget):
    """用 QPainter 画五连杆等效 2R 机构图"""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.theta_a = 0.0
        self.theta_b = 0.0
        self.endpoint = Pose(0, 0)
        self.setMinimumSize(PANEL_W, PANEL_H)

    def update_angles(self, theta_a: float, theta_b: float):
        self.theta_a = theta_a
        self.theta_b = theta_b
        self.endpoint = lk_forward(JointAngles(theta_a, theta_b))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 背景
        p.fillRect(self.rect(), QColor(28, 28, 30))

        # 工作空间环
        pen_ws = QPen(QColor(50, 50, 55), 1, Qt.PenStyle.DotLine)
        p.setPen(pen_ws)
        p.setBrush(Qt.BrushStyle.NoBrush)
        inner_r = WORKSPACE_MIN * SCALE
        outer_r = WORKSPACE_MAX * SCALE
        origin_px = QPointF(ORIGIN_X, ORIGIN_Y)
        p.drawEllipse(origin_px, inner_r, inner_r)
        p.drawEllipse(origin_px, outer_r, outer_r)

        # 机构点
        P1 = QPointF(ORIGIN_X, ORIGIN_Y)  # 固定铰链
        P2 = to_pixel(L1_MM * math.cos(self.theta_a), L1_MM * math.sin(self.theta_a))
        P7 = to_pixel(self.endpoint.x_mm, self.endpoint.y_mm)

        # 地面标记
        p.setPen(QPen(QColor(60, 60, 65), 2))
        p.drawLine(
            QPointF(ORIGIN_X - 80, ORIGIN_Y + 5),
            QPointF(ORIGIN_X + 80, ORIGIN_Y + 5),
        )
        # 地面阴影
        ground_y = ORIGIN_Y + 5
        shadow = QPointF(P7.x(), min(P7.y() + 30, ground_y + 60))
        p.setPen(QPen(QColor(40, 40, 42), 1))
        p.drawLine(P7, QPointF(P7.x(), ground_y + 30))

        # 连杆 (粗线)
        pen_link = QPen(QColor(100, 200, 255), 3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        p.setPen(pen_link)
        p.drawLine(P1, P2)
        pen_link2 = QPen(QColor(255, 160, 80), 3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
        p.setPen(pen_link2)
        p.drawLine(P2, P7)

        # 关节 (圆)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor(255, 255, 255)))
        p.drawEllipse(P1, 5, 5)
        p.drawEllipse(P2, 4, 4)
        p.setBrush(QBrush(QColor(255, 200, 50)))
        p.drawEllipse(P7, 6, 6)

        # 标题
        font = QFont("Monospace", 12, QFont.Weight.Bold)
        p.setFont(font)
        p.setPen(QColor(200, 200, 200))
        p.drawText(QRectF(10, 10, PANEL_W - 20, 24), Qt.AlignmentFlag.AlignLeft, self.title)

        # 角度和末端坐标
        font_info = QFont("Monospace", 9)
        p.setFont(font_info)
        p.setPen(QColor(150, 150, 155))
        a_deg = math.degrees(self.theta_a)
        b_deg = math.degrees(self.theta_b)
        info_lines = [
            f"θa: {a_deg:+.1f}°",
            f"θb: {b_deg:+.1f}°",
            f"P7: ({self.endpoint.x_mm:+.1f}, {self.endpoint.y_mm:+.1f}) mm",
            f"h: {abs(self.endpoint.y_mm):.1f} mm",
        ]
        for i, line in enumerate(info_lines):
            p.drawText(
                QRectF(10, 36 + i * 18, PANEL_W - 20, 18),
                Qt.AlignmentFlag.AlignLeft,
                line,
            )

        p.end()


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class TwinDisplay(QWidget):
    def __init__(self, udp_host: str, udp_port: int, csv_path: str | None = None):
        super().__init__()
        self.setWindowTitle("Unit5 Twin Display — 数字孪生")
        self.setStyleSheet("background-color: #1c1c1e;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        # 双面板
        panels = QHBoxLayout()
        self.left_panel = LegPanel("Left Leg (M1/M2)")
        self.right_panel = LegPanel("Right Leg (M3/M4)")
        panels.addWidget(self.left_panel)
        panels.addWidget(self.right_panel)
        layout.addLayout(panels)

        # 状态栏
        self.status = QLabel("等待数据...")
        self.status.setFont(QFont("Monospace", 10))
        self.status.setStyleSheet("color: #888; padding: 4px;")
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status)

        # 数据源
        self.csv_player: CsvPlayer | None = None
        self.udp: UdpReceiver | None = None
        if csv_path:
            self.csv_player = CsvPlayer(csv_path)
        else:
            self.udp = UdpReceiver(udp_host, udp_port)
            self.udp.start()

        # 刷新定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)  # ~30 fps

        self.frame_count = 0
        self.start_time = time.time()

    def _tick(self):
        if self.udp is not None:
            t_ms, m1, m2, m3, m4, count = self.udp.get_latest()
            self.frame_count = count
        elif self.csv_player is not None:
            row = self.csv_player.get_next()
            if row is None:
                return
            t_ms, m1, m2, m3, m4, count = row
            self.frame_count = count
        else:
            return

        # 电机角度 → 机构帧关节角
        left_a = motor_to_theta(m1, LEFT_OFFSET_A)
        left_b = motor_to_theta(m2, LEFT_OFFSET_B)
        right_a = motor_to_theta(m3, RIGHT_OFFSET_A)
        right_b = motor_to_theta(m4, RIGHT_OFFSET_B)

        self.left_panel.update_angles(left_a, left_b)
        self.right_panel.update_angles(right_a, right_b)

        elapsed = time.time() - self.start_time
        rate = self.frame_count / elapsed if elapsed > 0 else 0
        self.status.setText(
            f"t={t_ms}ms  |  {self.frame_count} 帧  {rate:.0f} Hz  |  "
            f"M1={m1:+.3f} M2={m2:+.3f} M3={m3:+.3f} M4={m4:+.3f} rad"
        )

    def closeEvent(self, event):
        if self.udp is not None:
            self.udp.running = False
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Unit5 数字孪生可视化")
    p.add_argument("--host", default="127.0.0.1", help="UDP 监听地址 (默认 127.0.0.1)")
    p.add_argument("--port", type=int, default=9999, help="UDP 监听端口 (默认 9999)")
    p.add_argument("--csv", help="离线回放 CSV 文件")
    args = p.parse_args()

    app = QApplication(sys.argv)
    window = TwinDisplay(args.host, args.port, args.csv)
    window.resize(PANEL_W * 2 + 30, PANEL_H + 60)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
