"""Almacenamiento en memoria, gestión de objetivos, escalones y exportación CSV.

Módulo puro en Python + NumPy (sin dependencias de Qt ni ROS) para mantener
la lógica de buffers de telemetría, cálculo de errores y generación de curvas
escalonadas fácilmente verificable con pruebas unitarias.
"""

from __future__ import annotations

import csv
import math
import time
from collections import deque
from itertools import islice
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

DEFAULT_MAX_SAMPLES = 50_000
CSV_HEADER = ("t", "az", "el", "az_obj", "el_obj", "err_az", "err_el")


def build_step_series(
    t: np.ndarray,
    y: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Convierte una serie (t, y) en una curva escalón (zero-order hold) segura ante NaN.

    Devuelve dos arrays vacíos si no hay ningún valor finito en ``y`` (así no se
    dibuja la curva mientras el objetivo no esté definido).
    """
    n = len(t)
    if n == 0 or len(y) != n or not np.any(np.isfinite(y)):
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64)

    if n == 1:
        return t.copy(), y.copy()

    t_step = np.empty(2 * n - 1, dtype=np.float64)
    y_step = np.empty(2 * n - 1, dtype=np.float64)

    t_step[0::2] = t
    t_step[1::2] = t[1:]

    y_step[0::2] = y
    y_step[1::2] = y[:-1]

    return t_step, y_step


class TelemetryStore:
    """Gestiona muestras de telemetría en buffers acotados y estado de objetivos."""

    def __init__(
        self,
        max_samples: int = DEFAULT_MAX_SAMPLES,
        start_time: Optional[float] = None,
    ) -> None:
        self._start_time: float = (
            time.monotonic() if start_time is None else float(start_time)
        )
        self._max_samples = max_samples

        self._t: deque[float] = deque(maxlen=max_samples)
        self._az: deque[float] = deque(maxlen=max_samples)
        self._el: deque[float] = deque(maxlen=max_samples)
        self._az_obj: deque[float] = deque(maxlen=max_samples)
        self._el_obj: deque[float] = deque(maxlen=max_samples)
        self._err_az: deque[float] = deque(maxlen=max_samples)
        self._err_el: deque[float] = deque(maxlen=max_samples)

        self.az_current: Optional[float] = None
        self.el_current: Optional[float] = None
        self.az_target: Optional[float] = None
        self.el_target: Optional[float] = None

    @property
    def sample_count(self) -> int:
        return len(self._t)

    @property
    def current_errors(self) -> Tuple[Optional[float], Optional[float]]:
        """Devuelve (err_az, err_el) = (objetivo - actual), o None si falta dato."""
        err_az = (
            self.az_target - self.az_current
            if self.az_target is not None and self.az_current is not None
            else None
        )
        err_el = (
            self.el_target - self.el_current
            if self.el_target is not None and self.el_current is not None
            else None
        )
        return err_az, err_el

    def elapsed_seconds(self, now: Optional[float] = None) -> float:
        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self._start_time)

    def record_rx(
        self,
        az: float,
        el: float,
        now: Optional[float] = None,
    ) -> Tuple[float, Optional[float], Optional[float]]:
        """Guarda una muestra RX (AZ, EL) con t en segundos desde el arranque.

        Devuelve ``(t, err_az, err_el)``.
        """
        t = self.elapsed_seconds(now)
        self.az_current = float(az)
        self.el_current = float(el)

        err_az, err_el = self.current_errors

        self._t.append(t)
        self._az.append(self.az_current)
        self._el.append(self.el_current)
        self._az_obj.append(math.nan if self.az_target is None else self.az_target)
        self._el_obj.append(math.nan if self.el_target is None else self.el_target)
        self._err_az.append(math.nan if err_az is None else err_az)
        self._err_el.append(math.nan if err_el is None else err_el)

        return t, err_az, err_el

    def apply_tx_command(self, command: str) -> bool:
        """Actualiza objetivos según un comando enviado en /satelite/cmd.

        Reglas:
        - A<x>      -> objetivo de azimut = x
        - E<y>      -> objetivo de elevación = y
        - G<x>,<y>  -> objetivo de azimut = x, objetivo de elevación = y
        - S         -> objetivo pasa a ser la posición actual (si existe)
        - Z         -> objetivo de azimut = 0.0
        - X         -> objetivo de elevación = 0.0
        - P/I/D     -> no modifican objetivos de posición

        Devuelve True si algún objetivo se actualizó.
        """
        if not command:
            return False

        prefix = command[0]
        payload = command[1:]

        if prefix == "A":
            self.az_target = float(payload)
            return True
        if prefix == "E":
            self.el_target = float(payload)
            return True
        if prefix == "G":
            az_str, el_str = payload.split(",", 1)
            self.az_target = float(az_str)
            self.el_target = float(el_str)
            return True
        if prefix == "S":
            changed = False
            if self.az_current is not None:
                self.az_target = self.az_current
                changed = True
            if self.el_current is not None:
                self.el_target = self.el_current
                changed = True
            return changed
        if prefix == "Z":
            self.az_target = 0.0
            return True
        if prefix == "X":
            self.el_target = 0.0
            return True
        return False

    def set_targets_from_api(self, az_deg: float, el_deg: float) -> bool:
        """Actualiza objetivos desde /satelite/look_angles (azimut y elevación en grados).

        Devuelve True si cambió al menos uno de los valores respecto al objetivo previo.
        """
        if not (math.isfinite(az_deg) and math.isfinite(el_deg)):
            return False

        new_az = float(az_deg)
        new_el = float(el_deg)
        changed = (
            self.az_target is None
            or self.el_target is None
            or abs(self.az_target - new_az) > 1e-9
            or abs(self.el_target - new_el) > 1e-9
        )
        self.az_target = new_az
        self.el_target = new_el
        return changed

    def set_targets_from_twist(self, angular_z_deg: float, angular_y_deg: float) -> bool:
        """Alias de compatibilidad para actualizar objetivos (azimut, elevación)."""
        return self.set_targets_from_api(angular_z_deg, angular_y_deg)

    def clear_samples(self) -> None:
        """Limpia únicamente los buffers de muestras (mantiene tiempo t0 y objetivos)."""
        self._t.clear()
        self._az.clear()
        self._el.clear()
        self._az_obj.clear()
        self._el_obj.clear()
        self._err_az.clear()
        self._err_el.clear()

    def get_window_arrays(
        self,
        window_seconds: Optional[float] = None,
    ) -> dict[str, np.ndarray]:
        """Obtiene arrays NumPy para la ventana temporal solicitada (o todos si None)."""
        n = len(self._t)
        if n == 0:
            empty = np.empty(0, dtype=np.float64)
            return {
                "t": empty,
                "az": empty,
                "el": empty,
                "az_obj": empty,
                "el_obj": empty,
                "err_az": empty,
                "err_el": empty,
            }

        t_all = np.fromiter(self._t, dtype=np.float64, count=n)
        start = 0
        if window_seconds is not None and window_seconds > 0:
            t_min = t_all[-1] - window_seconds
            idx = int(np.searchsorted(t_all, t_min, side="left"))
            # Incluir un punto anterior al borde izquierdo para continuidad visual.
            start = max(0, idx - 1)

        count = n - start
        if start == 0:
            return {
                "t": t_all,
                "az": np.fromiter(self._az, dtype=np.float64, count=n),
                "el": np.fromiter(self._el, dtype=np.float64, count=n),
                "az_obj": np.fromiter(self._az_obj, dtype=np.float64, count=n),
                "el_obj": np.fromiter(self._el_obj, dtype=np.float64, count=n),
                "err_az": np.fromiter(self._err_az, dtype=np.float64, count=n),
                "err_el": np.fromiter(self._err_el, dtype=np.float64, count=n),
            }

        return {
            "t": t_all[start:],
            "az": np.fromiter(islice(self._az, start, None), dtype=np.float64, count=count),
            "el": np.fromiter(islice(self._el, start, None), dtype=np.float64, count=count),
            "az_obj": np.fromiter(islice(self._az_obj, start, None), dtype=np.float64, count=count),
            "el_obj": np.fromiter(islice(self._el_obj, start, None), dtype=np.float64, count=count),
            "err_az": np.fromiter(islice(self._err_az, start, None), dtype=np.float64, count=count),
            "err_el": np.fromiter(islice(self._err_el, start, None), dtype=np.float64, count=count),
        }

    def export_csv(self, file_path: str | Path) -> int:
        """Exporta todas las muestras almacenadas a CSV con columnas estándar.

        Columnas: t, az, el, az_obj, el_obj, err_az, err_el.
        Los valores indefinidos (NaN) se escriben como celda vacía.
        Devuelve el número de filas de datos exportadas.
        """
        path = Path(file_path)

        def _fmt(val: float) -> str:
            return "" if math.isnan(val) else f"{val:.4f}"

        rows = zip(
            self._t,
            self._az,
            self._el,
            self._az_obj,
            self._el_obj,
            self._err_az,
            self._err_el,
        )
        count = 0
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(CSV_HEADER)
            for t, az, el, az_obj, el_obj, err_az, err_el in rows:
                writer.writerow(
                    (
                        f"{t:.4f}",
                        f"{az:.4f}",
                        f"{el:.4f}",
                        _fmt(az_obj),
                        _fmt(el_obj),
                        _fmt(err_az),
                        _fmt(err_el),
                    )
                )
                count += 1
        return count
