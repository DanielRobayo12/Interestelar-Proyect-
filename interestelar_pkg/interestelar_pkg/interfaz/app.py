#!/usr/bin/env python3
"""Aplicación de escritorio PySide6 + pyqtgraph + ROS 2 Jazzy para control satelital.

Nodo ROS 2:
- Publica comandos ``std_msgs/msg/String`` en ``/satelite/cmd``.
- Publica el ID NORAD ``std_msgs/msg/Int32`` en ``/satelite/norad_id``.
- Se suscribe a ``/satelite/rx`` (``std_msgs/msg/String``, líneas del PIC).
- Se suscribe a ``/satelite/info`` (``std_msgs/msg/String``, JSON ``{"satid", "satname"}``).
- Se suscribe a ``/satelite/look_angles`` (``std_msgs/msg/Float64MultiArray``,
  ``[azimut_deg, elevacion_deg, eclipsado]``).
- ``rclpy.spin`` corre en un ``QThread`` y se comunica con la UI únicamente
  mediante señales de Qt.
"""

from __future__ import annotations

import html
import math
import os
import queue
import signal
import sys
import threading
from datetime import datetime
from typing import Optional

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")

import numpy as np
import pyqtgraph as pg
import rclpy
from PySide6.QtCore import QByteArray, QLocale, Qt, QThread, QTimer, Signal, Slot
from PySide6.QtGui import QCloseEvent, QFont, QTextCursor
from PySide6.QtSvgWidgets import QSvgWidget
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray, Int32, String

if __package__:
    from .protocol import (
        MAX_LINE_LENGTH,
        ProtocolError,
        build_go_command,
        build_single_value_command,
        build_standalone_command,
        parse_satellite_info,
        parse_telemetry_line,
        validate_command,
        validate_norad_id,
    )
    from .satelite_svg import SVG_SATELITE
    from .telemetry import TelemetryStore, build_step_series
else:
    from protocol import (
        MAX_LINE_LENGTH,
        ProtocolError,
        build_go_command,
        build_single_value_command,
        build_standalone_command,
        parse_satellite_info,
        parse_telemetry_line,
        validate_command,
        validate_norad_id,
    )
    from satelite_svg import SVG_SATELITE
    from telemetry import TelemetryStore, build_step_series

CMD_TOPIC = "/satelite/cmd"
RX_TOPIC = "/satelite/rx"
NORAD_TOPIC = "/satelite/norad_id"
INFO_TOPIC = "/satelite/info"
LOOK_ANGLES_TOPIC = "/satelite/look_angles"
NODE_NAME = "satelite_gui_node"

PLOT_REFRESH_HZ = 20
PLOT_REFRESH_MS = int(1000 / PLOT_REFRESH_HZ)

WINDOW_OPTIONS: tuple[tuple[str, Optional[float]], ...] = (
    ("30 s", 30.0),
    ("60 s", 60.0),
    ("5 min", 300.0),
    ("Todo", None),
)

QUICK_SATELLITES: tuple[tuple[str, int], ...] = (
    ("ISS", 25544),
    ("Hubble", 20580),
    ("NOAA 19", 33591),
)

DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0b1120;
    color: #e2e8f0;
    font-size: 13px;
}
QGroupBox {
    background-color: #111827;
    border: 1px solid #1e293b;
    border-radius: 6px;
    margin-top: 10px;
    padding: 10px 8px 8px 8px;
    font-weight: bold;
    color: #cbd5e1;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: #94a3b8;
}
QLabel {
    background-color: transparent;
}
QDoubleSpinBox, QComboBox, QLineEdit {
    background-color: #0f172a;
    color: #f1f5f9;
    border: 1px solid #334155;
    border-radius: 4px;
    padding: 4px 8px;
}
QDoubleSpinBox:focus, QComboBox:focus, QLineEdit:focus {
    border: 1px solid #38bdf8;
}
QComboBox::drop-down {
    border: none;
}
QComboBox QAbstractItemView {
    background-color: #0f172a;
    color: #f1f5f9;
    selection-background-color: #1d4ed8;
    border: 1px solid #334155;
}
QPushButton {
    background-color: #1e293b;
    color: #f1f5f9;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 5px 12px;
    font-weight: 500;
}
QPushButton:hover {
    background-color: #334155;
    border-color: #475569;
}
QPushButton:pressed {
    background-color: #0f172a;
}
QTabWidget::pane {
    border: 1px solid #1e293b;
    border-radius: 6px;
    background-color: #0b1120;
    padding: 4px;
}
QTabBar::tab {
    background-color: #0f172a;
    color: #94a3b8;
    border: 1px solid #1e293b;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 7px 18px;
    margin-right: 4px;
    font-weight: 600;
}
QTabBar::tab:selected {
    background-color: #111827;
    color: #38bdf8;
    border-top: 2px solid #38bdf8;
}
QTabBar::tab:hover:!selected {
    color: #e2e8f0;
    background-color: #162033;
}
QSplitter::handle {
    background-color: #1e293b;
    height: 4px;
    width: 4px;
}
QSplitter::handle:hover {
    background-color: #38bdf8;
}
QStatusBar {
    background-color: #0f172a;
    color: #94a3b8;
    border-top: 1px solid #1e293b;
}
"""


class SateliteRosNode(Node):
    """Nodo ROS 2 para control PIC, selección NORAD y telemetría/API satelital."""

    def __init__(
        self,
        on_rx,
        on_sat_info,
        on_look_angles,
        on_drain_tx,
        node_name: str = NODE_NAME,
        cmd_topic: str = CMD_TOPIC,
        rx_topic: str = RX_TOPIC,
        norad_topic: str = NORAD_TOPIC,
        info_topic: str = INFO_TOPIC,
        look_angles_topic: str = LOOK_ANGLES_TOPIC,
    ) -> None:
        super().__init__(node_name)
        self._cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self._norad_pub = self.create_publisher(Int32, norad_topic, 10)

        self._rx_sub = self.create_subscription(String, rx_topic, self._on_rx_msg, 10)
        self._info_sub = self.create_subscription(
            String, info_topic, self._on_info_msg, 10
        )
        self._look_angles_sub = self.create_subscription(
            Float64MultiArray, look_angles_topic, self._on_look_angles_msg, 10
        )

        self._on_rx = on_rx
        self._on_sat_info = on_sat_info
        self._on_look_angles = on_look_angles
        # Guard condition para despertar rclpy.spin inmediatamente al encolar TX.
        self.tx_guard_condition = self.create_guard_condition(on_drain_tx)

    def _on_rx_msg(self, msg: String) -> None:
        self._on_rx(msg.data)

    def _on_info_msg(self, msg: String) -> None:
        self._on_sat_info(msg.data)

    def _on_look_angles_msg(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            return
        az = float(msg.data[0])
        el = float(msg.data[1])
        eclipsed_val = float(msg.data[2])
        if not (math.isfinite(az) and math.isfinite(el) and math.isfinite(eclipsed_val)):
            return
        self._on_look_angles(az, el, eclipsed_val >= 0.5)

    def publish_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)

    def publish_norad_id(self, norad_id: int) -> None:
        msg = Int32()
        msg.data = int(norad_id)
        self._norad_pub.publish(msg)


class RosSpinThread(QThread):
    """Hilo dedicado para ejecutar rclpy.spin y comunicarse con la UI por señales Qt."""

    rx_received = Signal(str)
    tx_published = Signal(str)
    norad_published = Signal(int)
    sat_info_received = Signal(str)
    look_angles_received = Signal(float, float, bool)
    status_changed = Signal(str)
    error_occurred = Signal(str)

    def __init__(
        self,
        node_name: str = NODE_NAME,
        cmd_topic: str = CMD_TOPIC,
        rx_topic: str = RX_TOPIC,
        norad_topic: str = NORAD_TOPIC,
        info_topic: str = INFO_TOPIC,
        look_angles_topic: str = LOOK_ANGLES_TOPIC,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._node_name = node_name
        self._cmd_topic = cmd_topic
        self._rx_topic = rx_topic
        self._norad_topic = norad_topic
        self._info_topic = info_topic
        self._look_angles_topic = look_angles_topic
        self._tx_queue: queue.SimpleQueue[tuple[str, object]] = queue.SimpleQueue()
        self._lock = threading.Lock()
        self._node: Optional[SateliteRosNode] = None
        self._guard_condition = None

    def _wake_spin_thread(self) -> None:
        with self._lock:
            gc = self._guard_condition
        if gc is not None and rclpy.ok():
            try:
                gc.trigger()
            except Exception as exc:
                self.error_occurred.emit(f"Error al despertar el hilo ROS: {exc}")

    def enqueue_command(self, command: str) -> None:
        """Valida y encola un comando PIC para publicarlo en /satelite/cmd."""
        validated = validate_command(command)
        self._tx_queue.put(("cmd", validated))
        self._wake_spin_thread()

    def enqueue_norad_id(self, norad_id: int) -> None:
        """Valida y encola un ID NORAD para publicarlo en /satelite/norad_id."""
        validated = validate_norad_id(norad_id)
        self._tx_queue.put(("norad", validated))
        self._wake_spin_thread()

    def _handle_rx(self, line: str) -> None:
        self.rx_received.emit(line)

    def _handle_sat_info(self, raw_json: str) -> None:
        self.sat_info_received.emit(raw_json)

    def _handle_look_angles(self, az_deg: float, el_deg: float, eclipsed: bool) -> None:
        self.look_angles_received.emit(az_deg, el_deg, eclipsed)

    def _drain_tx_queue(self) -> None:
        """Publica mensajes pendientes; se ejecuta exclusivamente en el hilo de spin."""
        while True:
            try:
                kind, payload = self._tx_queue.get_nowait()
            except queue.Empty:
                break
            node = self._node
            if node is None or not rclpy.ok():
                continue
            try:
                if kind == "cmd":
                    cmd_str = str(payload)
                    node.publish_command(cmd_str)
                    self.tx_published.emit(cmd_str)
                elif kind == "norad":
                    norad_val = int(payload)
                    node.publish_norad_id(norad_val)
                    self.norad_published.emit(norad_val)
            except Exception as exc:
                self.error_occurred.emit(f"Error publicando '{payload}': {exc}")

    def run(self) -> None:
        node: Optional[SateliteRosNode] = None
        try:
            node = SateliteRosNode(
                on_rx=self._handle_rx,
                on_sat_info=self._handle_sat_info,
                on_look_angles=self._handle_look_angles,
                on_drain_tx=self._drain_tx_queue,
                node_name=self._node_name,
                cmd_topic=self._cmd_topic,
                rx_topic=self._rx_topic,
                norad_topic=self._norad_topic,
                info_topic=self._info_topic,
                look_angles_topic=self._look_angles_topic,
            )
            with self._lock:
                self._node = node
                self._guard_condition = node.tx_guard_condition

            self.status_changed.emit(
                f"ROS 2 activo — pub: {self._cmd_topic}, {self._norad_topic} | "
                f"sub: {self._rx_topic}, {self._info_topic}, {self._look_angles_topic}"
            )
            self._drain_tx_queue()

            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:
            self.error_occurred.emit(f"Error en el hilo ROS 2: {exc}")
        finally:
            with self._lock:
                self._guard_condition = None
                node_to_destroy = self._node
                self._node = None
            if node_to_destroy is not None:
                try:
                    node_to_destroy.destroy_node()
                except Exception:
                    pass
            self.status_changed.emit("Nodo ROS 2 detenido")

    def stop(self) -> None:
        """Detiene rclpy.spin de forma segura."""
        with self._lock:
            self._guard_condition = None
        rclpy.try_shutdown()


class MainWindow(QMainWindow):
    """Ventana principal con pestañas Control, Seguimiento y Gráficas + log fijo abajo."""

    def __init__(self, ros_thread: RosSpinThread) -> None:
        super().__init__()
        self._ros_thread = ros_thread
        self._c_locale = QLocale.c()
        self._telemetry = TelemetryStore()

        self._plots_paused = False
        self._plot_dirty = False
        self._log_count = 0

        # Estado del selector de satélite (/satelite/norad_id y /satelite/info)
        self._requested_norad_id: Optional[int] = None
        self._active_sat_id: Optional[int] = None
        self._active_sat_name: Optional[str] = None
        self._last_eclipsed: Optional[bool] = None
        self._has_look_angles = False

        self.setWindowTitle("Control Satelital: QT + ROS2")
        self.resize(1220, 820)
        self.setMinimumSize(940, 680)
        self.setStyleSheet(DARK_STYLESHEET)

        self._build_ui()
        self._connect_signals()
        self._update_go_preview()
        self._update_satellite_status_ui()

        # Redibujado periódico de gráficas a ~20 Hz (nunca por cada muestra RX).
        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(PLOT_REFRESH_MS)
        self._plot_timer.timeout.connect(self._on_plot_timer)
        self._plot_timer.start()

    def _make_spinbox(
        self,
        minimum: float,
        maximum: float,
        decimals: int,
        step: float,
        suffix: str = "",
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        # Locale C garantiza punto decimal ('.') aunque el sistema esté en español.
        spin.setLocale(self._c_locale)
        spin.setGroupSeparatorShown(False)
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        if suffix:
            spin.setSuffix(suffix)
        spin.setMinimumWidth(125)
        return spin

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 6)
        root_layout.setSpacing(8)

        # ------------------------------------------------------------------
        # Encabezado compacto con telemetría actual, errores y satélite activo
        # ------------------------------------------------------------------
        header_bar = self._build_telemetry_header()
        root_layout.addWidget(header_bar, stretch=0)

        # ------------------------------------------------------------------
        # Splitter vertical: Pestañas arriba | Log siempre visible abajo
        # ------------------------------------------------------------------
        main_splitter = QSplitter(Qt.Orientation.Vertical)
        main_splitter.setChildrenCollapsible(False)

        self._tabs = QTabWidget()
        self._tabs.addTab(self._build_control_tab(), "Control")
        self._tabs.addTab(self._build_tracking_tab(), "Seguimiento")
        self._tabs.addTab(self._build_plots_tab(), "Gráficas")

        log_panel = self._build_log_panel()

        main_splitter.addWidget(self._tabs)
        main_splitter.addWidget(log_panel)
        main_splitter.setStretchFactor(0, 5)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([610, 125])

        root_layout.addWidget(main_splitter, stretch=1)

        # Barra de estado
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("Iniciando nodo ROS 2...")

    def _build_telemetry_header(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet(
            "QFrame {"
            "  background-color: #111827;"
            "  border: 1px solid #1e293b;"
            "  border-radius: 6px;"
            "}"
        )
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(12)

        (
            self._az_value_label,
            self._az_err_label,
            self._az_obj_label,
        ) = self._create_indicator_card(layout, "AZIMUT ACTUAL", "#00e5ff")

        (
            self._el_value_label,
            self._el_err_label,
            self._el_obj_label,
        ) = self._create_indicator_card(layout, "ELEVACIÓN ACTUAL", "#22c55e")

        # Tarjeta resumen de satélite activo en el encabezado
        sat_card = QFrame()
        sat_card.setStyleSheet(
            "QFrame {"
            "  background-color: #0f172a;"
            "  border: 1px solid #1e293b;"
            "  border-radius: 6px;"
            "}"
        )
        sat_vbox = QVBoxLayout(sat_card)
        sat_vbox.setContentsMargins(12, 6, 12, 6)
        sat_vbox.setSpacing(2)

        sat_title = QLabel("SATÉLITE ACTIVO")
        sat_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sat_title.setStyleSheet(
            "color: #94a3b8; font-size: 10px; font-weight: bold; border: none;"
        )

        self._header_sat_label = QLabel("Sin confirmar")
        self._header_sat_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._header_sat_label.setStyleSheet(
            "color: #f1f5f9; font-size: 16px; font-weight: bold; border: none;"
        )

        self._header_sat_sub = QLabel("ID NORAD: --")
        self._header_sat_sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._header_sat_sub.setStyleSheet(
            "color: #94a3b8; font-size: 11px; font-family: monospace; border: none;"
        )

        self._last_rx_label = QLabel("Última RX: sin datos")
        self._last_rx_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._last_rx_label.setStyleSheet(
            "color: #64748b; font-size: 10px; border: none;"
        )

        sat_vbox.addWidget(sat_title)
        sat_vbox.addWidget(self._header_sat_label)
        sat_vbox.addWidget(self._header_sat_sub)
        sat_vbox.addWidget(self._last_rx_label)
        layout.addWidget(sat_card, stretch=1)

        return bar

    def _create_indicator_card(
        self,
        parent_layout: QHBoxLayout,
        title: str,
        accent_color: str,
    ) -> tuple[QLabel, QLabel, QLabel]:
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame {"
            "  background-color: #0f172a;"
            "  border: 1px solid #1e293b;"
            "  border-radius: 6px;"
            "}"
        )
        vbox = QVBoxLayout(frame)
        vbox.setContentsMargins(12, 6, 12, 6)
        vbox.setSpacing(2)

        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_lbl.setStyleSheet(
            "color: #94a3b8; font-size: 10px; font-weight: bold; border: none;"
        )

        value_lbl = QLabel("--.- °")
        value_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value_lbl.setStyleSheet(
            f"color: {accent_color}; font-size: 24px; font-weight: bold; "
            "font-family: monospace; border: none;"
        )

        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(12)

        err_lbl = QLabel("Error: --.- °")
        err_lbl.setStyleSheet(
            "color: #fbbf24; font-size: 12px; font-weight: bold; "
            "font-family: monospace; border: none;"
        )

        obj_lbl = QLabel("Obj API: --.- °")
        obj_lbl.setStyleSheet(
            "color: #94a3b8; font-size: 11px; font-family: monospace; border: none;"
        )

        metrics_row.addStretch(1)
        metrics_row.addWidget(err_lbl)
        metrics_row.addWidget(obj_lbl)
        metrics_row.addStretch(1)

        vbox.addWidget(title_lbl)
        vbox.addWidget(value_lbl)
        vbox.addLayout(metrics_row)
        parent_layout.addWidget(frame, stretch=1)
        return value_lbl, err_lbl, obj_lbl

    # ------------------------------------------------------------------
    # Pestaña 1: Control (posicionamiento, PID, S/Z/X)
    # ------------------------------------------------------------------
    def _build_control_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(14)

        left_col = QVBoxLayout()
        left_col.setSpacing(12)

        # (1) Posicionamiento A, E y G
        pos_group = QGroupBox("Posicionamiento manual (A, E, G)")
        pos_grid = QGridLayout(pos_group)
        pos_grid.setSpacing(10)

        self._az_spin = self._make_spinbox(
            -360.0, 360.0, decimals=2, step=1.0, suffix=" °"
        )
        self._el_spin = self._make_spinbox(
            -90.0, 180.0, decimals=2, step=1.0, suffix=" °"
        )

        self._btn_send_a = QPushButton("Enviar A")
        self._btn_send_e = QPushButton("Enviar E")
        self._btn_send_g = QPushButton("Enviar G")
        self._go_preview_label = QLabel("G0,0")
        self._go_preview_label.setStyleSheet(
            "font-family: monospace; color: #94a3b8; padding: 2px 6px;"
        )

        pos_grid.addWidget(QLabel("Azimut (A):"), 0, 0)
        pos_grid.addWidget(self._az_spin, 0, 1)
        pos_grid.addWidget(self._btn_send_a, 0, 2)

        pos_grid.addWidget(QLabel("Elevación (E):"), 1, 0)
        pos_grid.addWidget(self._el_spin, 1, 1)
        pos_grid.addWidget(self._btn_send_e, 1, 2)

        pos_grid.addWidget(QLabel("Conjunto (G):"), 2, 0)
        pos_grid.addWidget(self._go_preview_label, 2, 1)
        pos_grid.addWidget(self._btn_send_g, 2, 2)

        left_col.addWidget(pos_group)

        # (3) Botones S (parar), Z (cero azimut), X (cero elevación)
        quick_group = QGroupBox("Control Rápido (enviados solos)")
        quick_layout = QHBoxLayout(quick_group)
        quick_layout.setSpacing(10)

        self._btn_stop = QPushButton("S — PARAR")
        self._btn_stop.setMinimumHeight(44)
        self._btn_stop.setStyleSheet(
            "QPushButton {"
            "  background-color: #dc2626; color: white; font-weight: bold;"
            "  border: 1px solid #ef4444; border-radius: 5px; padding: 8px 14px;"
            "}"
            "QPushButton:hover { background-color: #b91c1c; }"
            "QPushButton:pressed { background-color: #991b1b; }"
        )

        self._btn_zero_az = QPushButton("Z — Cero Azimut")
        self._btn_zero_az.setMinimumHeight(44)

        self._btn_zero_el = QPushButton("X — Cero Elevación")
        self._btn_zero_el.setMinimumHeight(44)

        quick_layout.addWidget(self._btn_stop)
        quick_layout.addWidget(self._btn_zero_az)
        quick_layout.addWidget(self._btn_zero_el)

        left_col.addWidget(quick_group)
        left_col.addStretch(1)

        # (2) Ganancias P, I, D (solo azimut)
        right_col = QVBoxLayout()
        right_col.setSpacing(12)

        pid_group = QGroupBox("Ganancias PID (solo Azimut)")
        pid_grid = QGridLayout(pid_group)
        pid_grid.setSpacing(10)

        self._kp_spin = self._make_spinbox(0.0, 100000.0, decimals=4, step=0.1)
        self._ki_spin = self._make_spinbox(0.0, 100000.0, decimals=4, step=0.01)
        self._kd_spin = self._make_spinbox(0.0, 100000.0, decimals=4, step=0.01)

        self._btn_send_p = QPushButton("Enviar P")
        self._btn_send_i = QPushButton("Enviar I")
        self._btn_send_d = QPushButton("Enviar D")
        self._btn_send_pid_all = QPushButton("Enviar P, I y D")
        self._btn_send_pid_all.setMinimumHeight(34)

        pid_grid.addWidget(QLabel("Kp (P):"), 0, 0)
        pid_grid.addWidget(self._kp_spin, 0, 1)
        pid_grid.addWidget(self._btn_send_p, 0, 2)

        pid_grid.addWidget(QLabel("Ki (I):"), 1, 0)
        pid_grid.addWidget(self._ki_spin, 1, 1)
        pid_grid.addWidget(self._btn_send_i, 1, 2)

        pid_grid.addWidget(QLabel("Kd (D):"), 2, 0)
        pid_grid.addWidget(self._kd_spin, 2, 1)
        pid_grid.addWidget(self._btn_send_d, 2, 2)

        pid_grid.addWidget(self._btn_send_pid_all, 3, 0, 1, 3)

        right_col.addWidget(pid_group)
        right_col.addStretch(1)

        layout.addLayout(left_col, stretch=3)
        layout.addLayout(right_col, stretch=2)
        return tab

    # ------------------------------------------------------------------
    # Pestaña 2: Seguimiento (selector de satélite, API look_angles, SVG)
    # ------------------------------------------------------------------
    def _build_tracking_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(14)

        left_col = QVBoxLayout()
        left_col.setSpacing(12)

        # Panel 1: Satélite a seguir
        selector_group = QGroupBox("Satélite a seguir")
        selector_group.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum
        )
        selector_vbox = QVBoxLayout(selector_group)
        selector_vbox.setSpacing(10)

        input_row = QHBoxLayout()
        input_row.setSpacing(8)

        input_row.addWidget(QLabel("ID NORAD:"))
        self._norad_input = QLineEdit()
        self._norad_input.setPlaceholderText("Entero positivo (ej. 25544)")
        self._norad_input.setText("25544")
        self._norad_input.setMaxLength(10)
        input_row.addWidget(self._norad_input, stretch=1)

        self._btn_follow = QPushButton("Seguir")
        self._btn_follow.setMinimumWidth(110)
        self._btn_follow.setStyleSheet(
            "QPushButton {"
            "  background-color: #0284c7; color: white; font-weight: bold;"
            "  border: 1px solid #38bdf8; border-radius: 5px; padding: 6px 14px;"
            "}"
            "QPushButton:hover { background-color: #0369a1; }"
            "QPushButton:pressed { background-color: #075985; }"
        )
        input_row.addWidget(self._btn_follow)
        selector_vbox.addLayout(input_row)

        # Accesos rápidos: ISS, Hubble, NOAA 19
        quick_row = QHBoxLayout()
        quick_row.setSpacing(8)
        quick_label = QLabel("Accesos rápidos:")
        quick_label.setStyleSheet("color: #94a3b8; font-weight: normal;")
        quick_row.addWidget(quick_label)

        self._quick_sat_buttons: list[tuple[QPushButton, int]] = []
        for sat_name, norad_id in QUICK_SATELLITES:
            btn = QPushButton(f"{sat_name} ({norad_id})")
            quick_row.addWidget(btn)
            self._quick_sat_buttons.append((btn, norad_id))
        quick_row.addStretch(1)
        selector_vbox.addLayout(quick_row)

        # Estado del satélite activo (/satelite/info)
        info_card = QFrame()
        info_card.setMinimumHeight(108)
        info_card.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum
        )
        info_card.setStyleSheet(
            "QFrame {"
            "  background-color: #0f172a;"
            "  border: 1px solid #1e293b;"
            "  border-radius: 6px;"
            "}"
        )
        info_grid = QGridLayout(info_card)
        info_grid.setContentsMargins(12, 10, 12, 10)
        info_grid.setHorizontalSpacing(12)
        info_grid.setVerticalSpacing(8)

        lbl_name_k = QLabel("Satélite activo:")
        lbl_name_k.setStyleSheet("color: #94a3b8; font-weight: normal; border: none;")
        self._active_sat_name_label = QLabel("--")
        self._active_sat_name_label.setMinimumHeight(22)
        self._active_sat_name_label.setStyleSheet(
            "color: #f1f5f9; font-size: 14px; font-weight: bold; border: none;"
        )

        lbl_id_k = QLabel("ID NORAD activo:")
        lbl_id_k.setStyleSheet("color: #94a3b8; font-weight: normal; border: none;")
        self._active_sat_id_label = QLabel("--")
        self._active_sat_id_label.setMinimumHeight(22)
        self._active_sat_id_label.setStyleSheet(
            "color: #38bdf8; font-size: 14px; font-weight: bold; "
            "font-family: monospace; border: none;"
        )

        lbl_status_k = QLabel("Confirmación:")
        lbl_status_k.setStyleSheet("color: #94a3b8; font-weight: normal; border: none;")
        self._sat_confirm_label = QLabel("Sin solicitud activa")
        self._sat_confirm_label.setMinimumHeight(22)
        self._sat_confirm_label.setStyleSheet(
            "color: #94a3b8; font-size: 13px; font-weight: bold; border: none;"
        )

        info_grid.addWidget(lbl_name_k, 0, 0)
        info_grid.addWidget(self._active_sat_name_label, 0, 1)
        info_grid.addWidget(lbl_id_k, 1, 0)
        info_grid.addWidget(self._active_sat_id_label, 1, 1)
        info_grid.addWidget(lbl_status_k, 2, 0)
        info_grid.addWidget(self._sat_confirm_label, 2, 1)
        info_grid.setRowMinimumHeight(0, 24)
        info_grid.setRowMinimumHeight(1, 22)
        info_grid.setRowMinimumHeight(2, 22)
        info_grid.setColumnStretch(1, 1)

        selector_vbox.addWidget(info_card)
        left_col.addWidget(selector_group)

        # Panel 2: Posición del satélite (API) (/satelite/look_angles)
        api_group = QGroupBox("Posición del satélite (API)")
        api_group.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum
        )
        api_vbox = QVBoxLayout(api_group)
        api_vbox.setSpacing(8)

        api_cards = QHBoxLayout()
        api_cards.setSpacing(10)

        self._api_az_label, _ = self._create_api_metric_card(
            api_cards, "AZIMUT (API)", "--.- °", "#ffb703"
        )
        self._api_el_label, self._api_el_horizon_label = self._create_api_metric_card(
            api_cards, "ELEVACIÓN (API)", "--.- °", "#e879f9", with_subtitle=True
        )
        self._api_eclipsed_label, _ = self._create_api_metric_card(
            api_cards, "ECLIPSADO", "--", "#94a3b8"
        )
        api_vbox.addLayout(api_cards)

        self._api_timestamp_label = QLabel(
            f"Última lectura ({LOOK_ANGLES_TOPIC}): sin datos"
        )
        self._api_timestamp_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self._api_timestamp_label.setStyleSheet("color: #64748b; font-size: 11px;")
        api_vbox.addWidget(self._api_timestamp_label)

        left_col.addWidget(api_group)
        left_col.addStretch(1)

        # Panel lateral derecho: Imagen SVG del satélite (240x156, proporción 400:260)
        visual_group = QGroupBox("Satélite en órbita")
        visual_vbox = QVBoxLayout(visual_group)
        visual_vbox.setContentsMargins(16, 16, 16, 14)
        visual_vbox.setSpacing(10)
        visual_vbox.setAlignment(Qt.AlignmentFlag.AlignCenter)

        svg_frame = QFrame()
        svg_frame.setStyleSheet(
            "QFrame {"
            "  background-color: #070d1a;"
            "  border: 1px solid #1e293b;"
            "  border-radius: 8px;"
            "}"
        )
        svg_frame_layout = QVBoxLayout(svg_frame)
        svg_frame_layout.setContentsMargins(12, 12, 12, 12)
        svg_frame_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._svg_widget = QSvgWidget()
        self._svg_widget.load(QByteArray(SVG_SATELITE.encode("utf-8")))
        self._svg_widget.setFixedSize(240, 156)
        self._svg_widget.setStyleSheet("background: transparent; border: none;")
        svg_frame_layout.addWidget(self._svg_widget, alignment=Qt.AlignmentFlag.AlignCenter)

        self._svg_caption_label = QLabel("En espera de satélite activo")
        self._svg_caption_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._svg_caption_label.setStyleSheet(
            "color: #cbd5e1; font-size: 13px; font-weight: bold;"
        )

        self._svg_eclipse_hint = QLabel("Iluminación: sin datos de API")
        self._svg_eclipse_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._svg_eclipse_hint.setStyleSheet("color: #64748b; font-size: 11px;")

        visual_vbox.addStretch(1)
        visual_vbox.addWidget(svg_frame, alignment=Qt.AlignmentFlag.AlignCenter)
        visual_vbox.addWidget(self._svg_caption_label)
        visual_vbox.addWidget(self._svg_eclipse_hint)
        visual_vbox.addStretch(1)

        layout.addLayout(left_col, stretch=3)
        layout.addWidget(visual_group, stretch=2)
        return tab

    def _create_api_metric_card(
        self,
        parent_layout: QHBoxLayout,
        title: str,
        initial_text: str,
        accent_color: str,
        with_subtitle: bool = False,
    ) -> tuple[QLabel, Optional[QLabel]]:
        frame = QFrame()
        frame.setMinimumHeight(82)
        frame.setStyleSheet(
            "QFrame {"
            "  background-color: #0f172a;"
            "  border: 1px solid #1e293b;"
            "  border-radius: 6px;"
            "}"
        )
        vbox = QVBoxLayout(frame)
        vbox.setContentsMargins(10, 8, 10, 8)
        vbox.setSpacing(3)

        title_lbl = QLabel(title)
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_lbl.setStyleSheet(
            "color: #94a3b8; font-size: 10px; font-weight: bold; border: none;"
        )

        value_lbl = QLabel(initial_text)
        value_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value_lbl.setStyleSheet(
            f"color: {accent_color}; font-size: 20px; font-weight: bold; "
            "font-family: monospace; border: none;"
        )

        vbox.addWidget(title_lbl)
        vbox.addWidget(value_lbl)

        subtitle_lbl: Optional[QLabel] = None
        if with_subtitle:
            subtitle_lbl = QLabel("")
            subtitle_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            subtitle_lbl.setFixedHeight(16)
            subtitle_lbl.setStyleSheet(
                "color: #fb923c; font-size: 11px; font-weight: bold; border: none;"
            )
            vbox.addWidget(subtitle_lbl)

        parent_layout.addWidget(frame, stretch=1)
        return value_lbl, subtitle_lbl

    # ------------------------------------------------------------------
    # Pestaña 3: Gráficas (posición, objetivo API y error)
    # ------------------------------------------------------------------
    def _build_plots_tab(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Barra de controles de las gráficas
        toolbar = QHBoxLayout()
        toolbar.setSpacing(8)

        toolbar.addWidget(QLabel("Ventana:"))
        self._window_combo = QComboBox()
        for label, seconds in WINDOW_OPTIONS:
            self._window_combo.addItem(label, seconds)
        self._window_combo.setCurrentIndex(0)  # 30 s
        self._window_combo.setMinimumWidth(85)
        toolbar.addWidget(self._window_combo)

        self._btn_pause_plots = QPushButton("Pausar")
        self._btn_pause_plots.setCheckable(True)
        self._btn_pause_plots.setMinimumWidth(92)
        toolbar.addWidget(self._btn_pause_plots)

        self._btn_clear_plots = QPushButton("Limpiar")
        toolbar.addWidget(self._btn_clear_plots)

        self._btn_export_csv = QPushButton("Exportar CSV")
        toolbar.addWidget(self._btn_export_csv)

        toolbar.addStretch(1)

        self._samples_label = QLabel("Muestras: 0")
        self._samples_label.setStyleSheet("color: #64748b; font-size: 11px;")
        toolbar.addWidget(self._samples_label)

        layout.addLayout(toolbar)

        # Configuración oscura de pyqtgraph
        pg.setConfigOptions(background="#0f172a", foreground="#cbd5e1", antialias=False)

        self._graphics_layout = pg.GraphicsLayoutWidget()
        self._graphics_layout.setStyleSheet(
            "border: 1px solid #1e293b; border-radius: 6px;"
        )
        self._graphics_layout.ci.layout.setVerticalSpacing(10)
        self._graphics_layout.ci.layout.setContentsMargins(8, 8, 12, 8)

        axis_width = 64

        def _configure_plot(plot_item, y_label: str, show_x_axis: bool = False) -> None:
            plot_item.setLabel("left", y_label)
            plot_item.showGrid(x=True, y=True, alpha=0.25)
            left_axis = plot_item.getAxis("left")
            left_axis.setWidth(axis_width)
            left_axis.enableAutoSIPrefix(False)
            bottom_axis = plot_item.getAxis("bottom")
            bottom_axis.enableAutoSIPrefix(False)
            if show_x_axis:
                plot_item.setLabel("bottom", "Tiempo (s)")
            else:
                bottom_axis.setStyle(showValues=False)
            try:
                plot_item.addLegend(offset=(10, 6), colCount=2)
            except TypeError:
                plot_item.addLegend(offset=(10, 6))

        # Gráfica 1: Azimut
        self._plot_az = self._graphics_layout.addPlot(row=0, col=0)
        _configure_plot(self._plot_az, "Az (°)", show_x_axis=False)

        self._curve_az = self._plot_az.plot(
            [],
            [],
            name="Actual",
            pen=pg.mkPen("#00e5ff", width=2.5),
            connect="finite",
        )
        self._curve_az_obj = self._plot_az.plot(
            [],
            [],
            name="Objetivo (API)",
            pen=pg.mkPen("#ffb703", width=2.2, style=Qt.PenStyle.DashLine),
            connect="finite",
        )

        # Gráfica 2: Elevación
        self._plot_el = self._graphics_layout.addPlot(row=1, col=0)
        _configure_plot(self._plot_el, "El (°)", show_x_axis=False)

        self._curve_el = self._plot_el.plot(
            [],
            [],
            name="Actual",
            pen=pg.mkPen("#22c55e", width=2.5),
            connect="finite",
        )
        self._curve_el_obj = self._plot_el.plot(
            [],
            [],
            name="Objetivo (API)",
            pen=pg.mkPen("#e879f9", width=2.2, style=Qt.PenStyle.DashLine),
            connect="finite",
        )

        # Gráfica 3: Error (Azimut y Elevación + línea horizontal en 0)
        self._plot_err = self._graphics_layout.addPlot(row=2, col=0)
        _configure_plot(self._plot_err, "Err (°)", show_x_axis=True)

        zero_line = pg.InfiniteLine(
            pos=0.0,
            angle=0,
            movable=False,
            pen=pg.mkPen("#64748b", width=1, style=Qt.PenStyle.DotLine),
        )
        self._plot_err.addItem(zero_line)

        self._curve_err_az = self._plot_err.plot(
            [],
            [],
            name="Error Az",
            pen=pg.mkPen("#38bdf8", width=2.5),
            connect="finite",
        )
        self._curve_err_el = self._plot_err.plot(
            [],
            [],
            name="Error El",
            pen=pg.mkPen("#ff4d6d", width=2.2, style=Qt.PenStyle.DashLine),
            connect="finite",
        )

        # Compartir eje X en segundos entre las tres gráficas apiladas
        self._plot_az.setXLink(self._plot_err)
        self._plot_el.setXLink(self._plot_err)

        layout.addWidget(self._graphics_layout, stretch=1)
        return panel

    # ------------------------------------------------------------------
    # Panel inferior siempre visible: Log de comunicación
    # ------------------------------------------------------------------
    def _build_log_panel(self) -> QGroupBox:
        log_group = QGroupBox("Log de comunicación")
        vbox = QVBoxLayout(log_group)
        vbox.setContentsMargins(8, 6, 8, 6)
        vbox.setSpacing(6)

        header = QHBoxLayout()
        header.setSpacing(8)

        self._log_count_label = QLabel("Mensajes: 0")
        self._log_count_label.setStyleSheet("color: #94a3b8; font-size: 11px;")

        rule_hint = QLabel(
            f"Reglas PIC: máx. {MAX_LINE_LENGTH} caracteres, punto decimal '.', "
            "sin dígito/+/- inicial, S/Z/X solos."
        )
        rule_hint.setStyleSheet("color: #64748b; font-size: 11px;")

        self._btn_clear_log = QPushButton("Limpiar log")
        self._btn_clear_log.setFixedHeight(26)

        header.addWidget(self._log_count_label)
        header.addWidget(rule_hint)
        header.addStretch(1)
        header.addWidget(self._btn_clear_log)
        vbox.addLayout(header)

        self._log_view = QTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.document().setMaximumBlockCount(2000)
        mono_font = QFont("Monospace")
        mono_font.setStyleHint(QFont.StyleHint.TypeWriter)
        mono_font.setPointSize(10)
        self._log_view.setFont(mono_font)
        self._log_view.setStyleSheet(
            "QTextEdit {"
            "  background-color: #0f172a; color: #e2e8f0;"
            "  border: 1px solid #1e293b; border-radius: 4px; padding: 6px;"
            "}"
        )
        self._log_view.setMinimumHeight(75)
        self._log_view.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        vbox.addWidget(self._log_view, stretch=1)

        return log_group

    def _connect_signals(self) -> None:
        # Señales del hilo ROS -> UI (exclusivamente Qt QueuedConnection)
        queued = Qt.ConnectionType.QueuedConnection
        self._ros_thread.rx_received.connect(self.on_rx_received, queued)
        self._ros_thread.tx_published.connect(self.on_tx_published, queued)
        self._ros_thread.norad_published.connect(self.on_norad_published, queued)
        self._ros_thread.sat_info_received.connect(self.on_sat_info_received, queued)
        self._ros_thread.look_angles_received.connect(
            self.on_look_angles_received, queued
        )
        self._ros_thread.status_changed.connect(self.on_status_changed, queued)
        self._ros_thread.error_occurred.connect(self.on_ros_error, queued)

        # Vista previa de G<az>,<el>
        self._az_spin.valueChanged.connect(self._update_go_preview)
        self._el_spin.valueChanged.connect(self._update_go_preview)

        # Pestaña Control — Posicionamiento
        self._btn_send_a.clicked.connect(self._send_azimut)
        self._btn_send_e.clicked.connect(self._send_elevacion)
        self._btn_send_g.clicked.connect(self._send_go)

        # Pestaña Control — Ganancias PID (solo azimut)
        self._btn_send_p.clicked.connect(
            lambda _checked=False: self._send_pid("P", self._kp_spin.value())
        )
        self._btn_send_i.clicked.connect(
            lambda _checked=False: self._send_pid("I", self._ki_spin.value())
        )
        self._btn_send_d.clicked.connect(
            lambda _checked=False: self._send_pid("D", self._kd_spin.value())
        )
        self._btn_send_pid_all.clicked.connect(self._send_pid_all)

        # Pestaña Control — Control rápido (S, Z, X)
        self._btn_stop.clicked.connect(
            lambda _checked=False: self._send_standalone("S")
        )
        self._btn_zero_az.clicked.connect(
            lambda _checked=False: self._send_standalone("Z")
        )
        self._btn_zero_el.clicked.connect(
            lambda _checked=False: self._send_standalone("X")
        )

        # Pestaña Seguimiento — Selector NORAD y accesos rápidos
        self._btn_follow.clicked.connect(self._send_norad_from_input)
        self._norad_input.returnPressed.connect(self._send_norad_from_input)
        for btn, norad_id in self._quick_sat_buttons:
            btn.clicked.connect(
                lambda _checked=False, nid=norad_id: self._select_quick_satellite(nid)
            )

        # Pestaña Gráficas
        self._window_combo.currentIndexChanged.connect(self._on_window_changed)
        self._btn_pause_plots.toggled.connect(self._on_pause_toggled)
        self._btn_clear_plots.clicked.connect(self._on_clear_plots)
        self._btn_export_csv.clicked.connect(self._on_export_csv)

        # Log
        self._btn_clear_log.clicked.connect(self._on_clear_log)

    # ------------------------------------------------------------------
    # Construcción y despacho de comandos PIC (/satelite/cmd)
    # ------------------------------------------------------------------
    def _dispatch_command(self, command: str) -> None:
        try:
            validated = validate_command(command)
            self._ros_thread.enqueue_command(validated)
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            self._status_bar.showMessage(f"Comando rechazado: {exc}", 5000)

    def _update_go_preview(self) -> None:
        try:
            cmd = build_go_command(
                self._az_spin.value(), self._el_spin.value(), max_decimals=2
            )
            self._go_preview_label.setText(f"{cmd} ({len(cmd)}/{MAX_LINE_LENGTH})")
            self._go_preview_label.setStyleSheet(
                "font-family: monospace; color: #94a3b8; padding: 2px 6px;"
            )
        except ProtocolError as exc:
            self._go_preview_label.setText("Excede límite")
            self._go_preview_label.setToolTip(str(exc))
            self._go_preview_label.setStyleSheet(
                "font-family: monospace; color: #f87171; padding: 2px 6px;"
            )

    @Slot()
    def _send_azimut(self) -> None:
        try:
            cmd = build_single_value_command("A", self._az_spin.value(), max_decimals=2)
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            return
        self._dispatch_command(cmd)

    @Slot()
    def _send_elevacion(self) -> None:
        try:
            cmd = build_single_value_command("E", self._el_spin.value(), max_decimals=2)
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            return
        self._dispatch_command(cmd)

    @Slot()
    def _send_go(self) -> None:
        try:
            cmd = build_go_command(
                self._az_spin.value(), self._el_spin.value(), max_decimals=2
            )
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            return
        self._dispatch_command(cmd)

    def _send_pid(self, prefix: str, value: float) -> None:
        try:
            cmd = build_single_value_command(prefix, value, max_decimals=4)
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            return
        self._dispatch_command(cmd)

    @Slot()
    def _send_pid_all(self) -> None:
        self._send_pid("P", self._kp_spin.value())
        self._send_pid("I", self._ki_spin.value())
        self._send_pid("D", self._kd_spin.value())

    def _send_standalone(self, prefix: str) -> None:
        try:
            cmd = build_standalone_command(prefix)
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            return
        self._dispatch_command(cmd)

    # ------------------------------------------------------------------
    # Selector de satélite (/satelite/norad_id y /satelite/info)
    # ------------------------------------------------------------------
    @Slot()
    def _send_norad_from_input(self) -> None:
        raw_text = self._norad_input.text()
        try:
            norad_id = validate_norad_id(raw_text)
        except ProtocolError as exc:
            self._append_log("ERR", str(exc))
            self._status_bar.showMessage(str(exc), 5000)
            return

        self._norad_input.setText(str(norad_id))
        self._requested_norad_id = norad_id
        self._update_satellite_status_ui()
        self._ros_thread.enqueue_norad_id(norad_id)

    def _select_quick_satellite(self, norad_id: int) -> None:
        self._norad_input.setText(str(norad_id))
        self._send_norad_from_input()

    def _update_satellite_status_ui(self) -> None:
        if self._active_sat_id is not None:
            active_name = self._active_sat_name or "(sin nombre)"
            self._active_sat_name_label.setText(active_name)
            self._active_sat_id_label.setText(str(self._active_sat_id))
            self._header_sat_label.setText(active_name)
            self._header_sat_sub.setText(f"ID NORAD: {self._active_sat_id}")
            self._svg_caption_label.setText(f"{active_name} (NORAD {self._active_sat_id})")
        elif self._has_look_angles:
            self._active_sat_name_label.setText("Datos recibidos (sin nombre)")
            self._active_sat_id_label.setText("--")
            self._header_sat_label.setText("Datos recibidos (sin nombre)")
            self._header_sat_sub.setText("ID NORAD: --")
            self._svg_caption_label.setText("Datos recibidos (sin nombre)")
        else:
            self._active_sat_name_label.setText("--")
            self._active_sat_id_label.setText("--")
            self._header_sat_label.setText("Sin satélite activo")
            self._header_sat_sub.setText("ID NORAD: --")
            self._svg_caption_label.setText("En espera de satélite activo")

        if self._requested_norad_id is None:
            if self._active_sat_id is not None:
                self._sat_confirm_label.setText("Activo")
                self._sat_confirm_label.setStyleSheet(
                    "color: #22c55e; font-size: 13px; font-weight: bold; border: none;"
                )
            else:
                self._sat_confirm_label.setText("Sin solicitud activa")
                self._sat_confirm_label.setStyleSheet(
                    "color: #94a3b8; font-size: 13px; font-weight: bold; border: none;"
                )
            return

        is_confirmed = (
            self._active_sat_id == self._requested_norad_id
            and bool(self._active_sat_name)
        )
        if is_confirmed:
            self._sat_confirm_label.setText(
                f"Confirmado (ID {self._requested_norad_id})"
            )
            self._sat_confirm_label.setStyleSheet(
                "color: #22c55e; font-size: 13px; font-weight: bold; border: none;"
            )
        else:
            self._sat_confirm_label.setText(
                f"esperando confirmación… (pedido: {self._requested_norad_id})"
            )
            self._sat_confirm_label.setStyleSheet(
                "color: #fbbf24; font-size: 13px; font-weight: bold; border: none;"
            )
            self._header_sat_sub.setText(
                f"esperando confirmación… ({self._requested_norad_id})"
            )

    # ------------------------------------------------------------------
    # Slots que reciben señales desde el hilo ROS 2
    # ------------------------------------------------------------------
    @Slot(str)
    def on_tx_published(self, command: str) -> None:
        self._append_log("TX", command)

    @Slot(int)
    def on_norad_published(self, norad_id: int) -> None:
        self._append_log("SAT", f"Solicitado NORAD ID -> {norad_id}")
        self._status_bar.showMessage(
            f"ID NORAD {norad_id} publicado en {NORAD_TOPIC}", 4000
        )

    @Slot(str)
    def on_sat_info_received(self, raw_json: str) -> None:
        parsed = parse_satellite_info(raw_json)
        if parsed is None:
            self._append_log("ERR", f"JSON inválido en {INFO_TOPIC}: {raw_json}")
            return

        satid, satname = parsed
        changed = satid != self._active_sat_id or satname != self._active_sat_name
        self._active_sat_id = satid
        self._active_sat_name = satname
        self._update_satellite_status_ui()

        if changed:
            self._append_log("INF", f"Satélite activo: {satname or '(sin nombre)'} ({satid})")

    @Slot(float, float, bool)
    def on_look_angles_received(
        self, az_deg: float, el_deg: float, eclipsed: bool
    ) -> None:
        # Los look angles de la API son el objetivo de las gráficas de error.
        self._telemetry.set_targets_from_api(az_deg, el_deg)

        self._api_az_label.setText(f"{az_deg:.2f} °")
        self._api_el_label.setText(f"{el_deg:.2f} °")
        if self._api_el_horizon_label is not None:
            self._api_el_horizon_label.setText(
                "Bajo el horizonte" if el_deg < 0.0 else ""
            )

        if not self._has_look_angles:
            self._has_look_angles = True
            self._update_satellite_status_ui()
        if eclipsed:
            self._api_eclipsed_label.setText("Sí")
            self._api_eclipsed_label.setStyleSheet(
                "color: #fbbf24; font-size: 20px; font-weight: bold; "
                "font-family: monospace; border: none;"
            )
            self._svg_eclipse_hint.setText("Iluminación: En eclipse (sombra)")
        else:
            self._api_eclipsed_label.setText("No")
            self._api_eclipsed_label.setStyleSheet(
                "color: #22c55e; font-size: 20px; font-weight: bold; "
                "font-family: monospace; border: none;"
            )
            self._svg_eclipse_hint.setText("Iluminación: Iluminado por el Sol")

        now_str = datetime.now().strftime("%H:%M:%S")
        self._api_timestamp_label.setText(
            f"Última lectura ({LOOK_ANGLES_TOPIC}): {now_str}"
        )

        self._update_target_and_error_indicators()

        if self._last_eclipsed is None or self._last_eclipsed != eclipsed:
            estado = "eclipsado" if eclipsed else "iluminado"
            self._append_log(
                "API",
                f"Look angles: AZ={az_deg:.2f}°, EL={el_deg:.2f}° ({estado})",
            )
            self._last_eclipsed = eclipsed

    @Slot(str)
    def on_rx_received(self, line: str) -> None:
        clean_line = line.rstrip("\r\n")
        self._append_log("RX", clean_line)

        parsed = parse_telemetry_line(clean_line)
        if parsed is not None:
            azimut, elevacion = parsed
            self._telemetry.record_rx(azimut, elevacion)
            self._az_value_label.setText(f"{azimut:.2f} °")
            self._el_value_label.setText(f"{elevacion:.2f} °")
            self._update_target_and_error_indicators()

            now_str = datetime.now().strftime("%H:%M:%S")
            self._last_rx_label.setText(f"Última RX: {now_str}")
            self._samples_label.setText(f"Muestras: {self._telemetry.sample_count}")
            self._plot_dirty = True

    @Slot(str)
    def on_status_changed(self, status: str) -> None:
        self._status_bar.showMessage(status)
        self._append_log("SYS", status)

    @Slot(str)
    def on_ros_error(self, message: str) -> None:
        self._status_bar.showMessage(message, 6000)
        self._append_log("ERR", message)

    # ------------------------------------------------------------------
    # Indicadores de error y refresco de gráficas a ~20 Hz
    # ------------------------------------------------------------------
    def _update_target_and_error_indicators(self) -> None:
        err_az, err_el = self._telemetry.current_errors

        if self._telemetry.az_target is not None:
            self._az_obj_label.setText(f"Obj API: {self._telemetry.az_target:.2f} °")
        else:
            self._az_obj_label.setText("Obj API: --.- °")

        if self._telemetry.el_target is not None:
            self._el_obj_label.setText(f"Obj API: {self._telemetry.el_target:.2f} °")
        else:
            self._el_obj_label.setText("Obj API: --.- °")

        if err_az is not None:
            self._az_err_label.setText(f"Error: {err_az:+.2f} °")
        else:
            self._az_err_label.setText("Error: --.- °")

        if err_el is not None:
            self._el_err_label.setText(f"Error: {err_el:+.2f} °")
        else:
            self._el_err_label.setText("Error: --.- °")

    @Slot()
    def _on_plot_timer(self) -> None:
        if self._plots_paused or not self._plot_dirty:
            return
        self._refresh_plots(update_x_range=True)

    def _refresh_plots(self, update_x_range: bool) -> None:
        self._plot_dirty = False
        window_seconds: Optional[float] = self._window_combo.currentData()
        data = self._telemetry.get_window_arrays(window_seconds=window_seconds)
        t = data["t"]

        if len(t) == 0:
            empty = np.empty(0, dtype=np.float64)
            self._curve_az.setData(empty, empty)
            self._curve_az_obj.setData(empty, empty)
            self._curve_el.setData(empty, empty)
            self._curve_el_obj.setData(empty, empty)
            self._curve_err_az.setData(empty, empty)
            self._curve_err_el.setData(empty, empty)
            return

        # Azimut: actual + objetivo API (escalón)
        self._curve_az.setData(t, data["az"], connect="finite")
        t_az_step, az_obj_step = build_step_series(t, data["az_obj"])
        self._curve_az_obj.setData(t_az_step, az_obj_step, connect="finite")

        # Elevación: actual + objetivo API (escalón)
        self._curve_el.setData(t, data["el"], connect="finite")
        t_el_step, el_obj_step = build_step_series(t, data["el_obj"])
        self._curve_el_obj.setData(t_el_step, el_obj_step, connect="finite")

        # Error: solo se dibuja si existe objetivo definido desde la API
        err_az = data["err_az"]
        if np.any(np.isfinite(err_az)):
            self._curve_err_az.setData(t, err_az, connect="finite")
        else:
            self._curve_err_az.setData([], [])

        err_el = data["err_el"]
        if np.any(np.isfinite(err_el)):
            self._curve_err_el.setData(t, err_el, connect="finite")
        else:
            self._curve_err_el.setData([], [])

        if update_x_range:
            if window_seconds is not None:
                x_max = max(float(t[-1]), 1.0)
                x_min = max(0.0, x_max - window_seconds)
            else:
                x_min = float(t[0])
                x_max = max(float(t[-1]), x_min + 1.0)
            self._plot_err.setXRange(x_min, x_max, padding=0.02)

    @Slot(int)
    def _on_window_changed(self, _index: int) -> None:
        self._refresh_plots(update_x_range=True)

    @Slot(bool)
    def _on_pause_toggled(self, paused: bool) -> None:
        self._plots_paused = paused
        if paused:
            self._btn_pause_plots.setText("Reanudar")
            self._btn_pause_plots.setStyleSheet(
                "QPushButton {"
                "  background-color: #d97706; color: #0f172a; font-weight: bold;"
                "  border: 1px solid #f59e0b; border-radius: 5px; padding: 5px 10px;"
                "}"
                "QPushButton:hover { background-color: #f59e0b; }"
            )
            self._status_bar.showMessage("Gráficas en pausa (puedes hacer zoom/pan)", 3000)
        else:
            self._btn_pause_plots.setText("Pausar")
            self._btn_pause_plots.setStyleSheet("")
            self._plot_az.enableAutoRange(axis="y", enable=True)
            self._plot_el.enableAutoRange(axis="y", enable=True)
            self._plot_err.enableAutoRange(axis="y", enable=True)
            self._plot_dirty = True
            self._refresh_plots(update_x_range=True)
            self._status_bar.showMessage("Gráficas reanudadas", 2000)

    @Slot()
    def _on_clear_plots(self) -> None:
        self._telemetry.clear_samples()
        self._samples_label.setText("Muestras: 0")
        self._refresh_plots(update_x_range=False)
        self._status_bar.showMessage("Historial de gráficas limpiado", 2500)

    @Slot()
    def _on_export_csv(self) -> None:
        if self._telemetry.sample_count == 0:
            QMessageBox.information(
                self,
                "Exportar CSV",
                "Todavía no hay muestras de telemetría para exportar.",
            )
            return

        default_name = (
            f"telemetria_satelite_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Exportar telemetría a CSV",
            default_name,
            "Archivos CSV (*.csv);;Todos los archivos (*)",
        )
        if not file_path:
            return
        if not file_path.lower().endswith(".csv"):
            file_path += ".csv"

        try:
            rows = self._telemetry.export_csv(file_path)
        except OSError as exc:
            QMessageBox.critical(
                self,
                "Error al exportar CSV",
                f"No se pudo guardar el archivo:\n{exc}",
            )
            return

        msg = f"CSV exportado ({rows} muestras): {file_path}"
        self._status_bar.showMessage(msg, 6000)
        self._append_log("SYS", msg)

    # ------------------------------------------------------------------
    # Log y cierre seguro
    # ------------------------------------------------------------------
    @Slot()
    def _on_clear_log(self) -> None:
        self._log_view.clear()
        self._log_count = 0
        self._log_count_label.setText("Mensajes: 0")

    def _append_log(self, tag: str, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        colors = {
            "TX": "#38bdf8",   # celeste (/satelite/cmd)
            "RX": "#4ade80",   # verde (/satelite/rx)
            "SAT": "#c084fc",  # violeta (/satelite/norad_id)
            "INF": "#22d3ee",  # cian (/satelite/info)
            "API": "#fbbf24",  # ámbar (/satelite/look_angles)
            "ERR": "#f87171",  # rojo
            "SYS": "#94a3b8",  # gris
        }
        arrows = {
            "TX": "->",
            "RX": "<-",
            "SAT": "->",
            "INF": "<-",
            "API": "<-",
            "ERR": "!!",
            "SYS": "--",
        }
        color = colors.get(tag, "#e2e8f0")
        arrow = arrows.get(tag, "  ")
        escaped_msg = html.escape(message)

        tag_padded = tag.ljust(3).replace(" ", "&nbsp;")
        entry_html = (
            f'<span style="color:#64748b;">[{timestamp}]</span> '
            f'<span style="color:{color}; font-weight:bold;">{tag_padded} {arrow}</span> '
            f'<span style="color:#f1f5f9;">{escaped_msg}</span>'
        )
        self._log_view.append(entry_html)
        self._log_view.moveCursor(QTextCursor.MoveOperation.End)

        self._log_count += 1
        self._log_count_label.setText(f"Mensajes: {self._log_count}")

    def closeEvent(self, event: QCloseEvent) -> None:
        self._plot_timer.stop()
        self._ros_thread.stop()
        if self._ros_thread.isRunning():
            self._ros_thread.wait(3000)
        event.accept()


def main() -> int:
    rclpy.init(args=sys.argv)
    qt_args = remove_ros_args(args=sys.argv)
    app = QApplication(qt_args)

    ros_thread = RosSpinThread()
    window = MainWindow(ros_thread)

    # Permitir cierre limpio con Ctrl+C desde la terminal.
    def _handle_sigint(_sig, _frame) -> None:
        window.close()

    signal.signal(signal.SIGINT, _handle_sigint)
    sigint_timer = QTimer()
    sigint_timer.start(200)
    sigint_timer.timeout.connect(lambda: None)

    window.show()
    ros_thread.start()

    try:
        return app.exec()
    finally:
        ros_thread.stop()
        if ros_thread.isRunning():
            ros_thread.wait(3000)
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
