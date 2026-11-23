"""Protocolo de comandos y parser de telemetría para el controlador PIC del satélite.

Reglas del protocolo:
- Longitud máxima de línea: 23 caracteres.
- Decimales siempre con punto ('.'), sin notación científica.
- Nunca iniciar un comando con un dígito, '+' o '-'.
- Los comandos S, Z y X se envían solos (sin argumentos).
- Comandos soportados:
    - A<grados>        : Azimut
    - E<grados>        : Elevación
    - G<az>,<el>       : Azimut y elevación simultáneos
    - P<val>, I<val>, D<val> : Ganancias PID de azimut
    - S                : Parar
    - Z                : Cero de azimut
    - X                : Cero de elevación
"""

from __future__ import annotations

import json
import math
import re
from typing import Optional, Tuple

MAX_LINE_LENGTH = 23
INT32_MAX = 2_147_483_647

STANDALONE_COMMANDS = frozenset({"S", "Z", "X"})
SINGLE_VALUE_COMMANDS = frozenset({"A", "E", "P", "I", "D"})
PAIR_VALUE_COMMANDS = frozenset({"G"})
ALLOWED_COMMANDS = STANDALONE_COMMANDS | SINGLE_VALUE_COMMANDS | PAIR_VALUE_COMMANDS

_NUMBER_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d+)?|\.\d+)$")
_TELEMETRY_RE = re.compile(
    r"^\s*AZ\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*,\s*EL\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*$"
)


class ProtocolError(ValueError):
    """Error lanzado cuando un comando incumple las reglas del protocolo del PIC."""


def format_number(value: float, max_decimals: int = 4) -> str:
    """Formatea un número real con punto decimal, sin ceros sobrantes ni notación científica."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"Valor numérico inválido: {value!r}")
    if not math.isfinite(value):
        raise ProtocolError("El valor numérico debe ser finito (no NaN ni infinito).")

    rounded = round(float(value), max_decimals)
    if rounded == 0.0:
        return "0"

    text = f"{rounded:.{max_decimals}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if text == "-0":
        return "0"
    return text


def validate_command(command: str) -> str:
    """Valida que una línea de comando cumpla todas las reglas del protocolo PIC."""
    if not isinstance(command, str) or not command:
        raise ProtocolError("El comando no puede estar vacío.")

    if not command.isascii():
        raise ProtocolError("El comando solo puede contener caracteres ASCII.")

    if any(ch.isspace() for ch in command):
        raise ProtocolError("El comando no puede contener espacios ni saltos de línea.")

    if len(command) > MAX_LINE_LENGTH:
        raise ProtocolError(
            f"El comando '{command}' tiene {len(command)} caracteres "
            f"(máximo permitido: {MAX_LINE_LENGTH})."
        )

    first = command[0]
    if first.isdigit() or first in "+-":
        raise ProtocolError(
            f"El comando '{command}' no puede empezar con un dígito, '+' o '-'."
        )

    if first not in ALLOWED_COMMANDS:
        raise ProtocolError(f"Prefijo de comando no reconocido: '{first}'.")

    if first in STANDALONE_COMMANDS:
        if len(command) != 1:
            raise ProtocolError(
                f"El comando '{first}' debe enviarse solo, sin argumentos."
            )
        return command

    payload = command[1:]
    if not payload:
        raise ProtocolError(f"El comando '{first}' requiere un valor numérico.")

    if first in SINGLE_VALUE_COMMANDS:
        if "," in payload:
            raise ProtocolError(
                f"El comando '{first}' debe usar punto decimal ('.'), no coma (',')."
            )
        if not _NUMBER_RE.match(payload):
            raise ProtocolError(
                f"Argumento numérico inválido para '{first}': '{payload}'."
            )
        return command

    if first in PAIR_VALUE_COMMANDS:
        parts = payload.split(",")
        if len(parts) != 2:
            raise ProtocolError(
                f"El comando 'G' debe tener formato G<az>,<el>: recibido '{command}'."
            )
        az_str, el_str = parts
        if not _NUMBER_RE.match(az_str) or not _NUMBER_RE.match(el_str):
            raise ProtocolError(
                f"Argumentos numéricos inválidos en '{command}'. Use punto decimal."
            )
        return command

    return command


def build_single_value_command(prefix: str, value: float, max_decimals: int = 4) -> str:
    """Construye y valida comandos tipo A<grados>, E<grados>, P<v>, I<v>, D<v>."""
    if prefix not in SINGLE_VALUE_COMMANDS:
        raise ProtocolError(f"Prefijo inválido para comando de un valor: '{prefix}'.")
    cmd = f"{prefix}{format_number(value, max_decimals=max_decimals)}"
    return validate_command(cmd)


def build_go_command(azimut: float, elevacion: float, max_decimals: int = 3) -> str:
    """Construye y valida el comando G<az>,<el>."""
    az_str = format_number(azimut, max_decimals=max_decimals)
    el_str = format_number(elevacion, max_decimals=max_decimals)
    cmd = f"G{az_str},{el_str}"
    return validate_command(cmd)


def build_standalone_command(prefix: str) -> str:
    """Construye y valida comandos sin argumentos: S, Z o X."""
    if prefix not in STANDALONE_COMMANDS:
        raise ProtocolError(
            f"Solo S, Z y X son comandos sin argumentos (recibido: '{prefix}')."
        )
    return validate_command(prefix)


def parse_telemetry_line(line: str) -> Optional[Tuple[float, float]]:
    """Parser aislado para líneas de telemetría recibidas desde el PIC.

    Formato actual esperado:
        AZ=<x>,EL=<y>

    Devuelve ``(azimut, elevacion)`` como floats si la línea coincide con el
    formato esperado, o ``None`` si la línea es otro tipo de mensaje (ACK,
    estado, diagnóstico, etc.).

    Esta función está aislada deliberadamente para facilitar cambios futuros
    en el formato de telemetría del PIC sin modificar la interfaz ni el nodo ROS.
    """
    if not isinstance(line, str):
        return None
    match = _TELEMETRY_RE.match(line)
    if not match:
        return None
    try:
        azimut = float(match.group(1))
        elevacion = float(match.group(2))
    except ValueError:
        return None
    if not (math.isfinite(azimut) and math.isfinite(elevacion)):
        return None
    return azimut, elevacion


def validate_norad_id(raw_value: int | str) -> int:
    """Valida que el ID NORAD sea un entero positivo compatible con std_msgs/Int32."""
    if isinstance(raw_value, bool):
        raise ProtocolError("El ID NORAD debe ser un entero positivo.")

    if isinstance(raw_value, int):
        norad_id = raw_value
    elif isinstance(raw_value, str):
        stripped = raw_value.strip()
        if not stripped or not stripped.isdigit():
            raise ProtocolError(
                f"ID NORAD inválido: '{raw_value}'. Debe ser un entero positivo."
            )
        norad_id = int(stripped)
    else:
        raise ProtocolError(
            f"ID NORAD inválido: {raw_value!r}. Debe ser un entero positivo."
        )

    if norad_id <= 0:
        raise ProtocolError("El ID NORAD debe ser mayor que 0.")
    if norad_id > INT32_MAX:
        raise ProtocolError(
            f"El ID NORAD excede el máximo permitido para Int32 ({INT32_MAX})."
        )
    return norad_id


def parse_satellite_info(raw_json: str) -> Optional[Tuple[int, str]]:
    """Parsea el JSON de /satelite/info con formato {"satid": ..., "satname": ...}.

    Devuelve ``(satid, satname)`` o ``None`` si el mensaje no es válido.
    """
    if not isinstance(raw_json, str) or not raw_json.strip():
        return None
    try:
        payload = json.loads(raw_json)
    except json.JSONDecodeError:
        return None
    if not isinstance(payload, dict):
        return None

    raw_id = payload.get("satid")
    raw_name = payload.get("satname")
    if raw_id is None or raw_name is None:
        return None

    try:
        satid = validate_norad_id(raw_id)
    except ProtocolError:
        return None

    satname = str(raw_name).strip()
    return satid, satname
