from __future__ import annotations

from dataclasses import dataclass
import csv
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(slots=True)
class TraceCommand:
    linear_mps: float
    angular_rad_s: float


@dataclass(slots=True)
class TraceReplayPilot:
    commands: list[TraceCommand]
    loop: bool = True
    index: int = 0

    @classmethod
    def from_csv(
        cls,
        path: str | Path,
        *,
        linear_col: str = 'linear_mps',
        angular_col: str = 'angular_rad_s',
        loop: bool = True,
    ) -> 'TraceReplayPilot':
        path = Path(path)
        commands: list[TraceCommand] = []
        with path.open('r', encoding='utf-8', newline='') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                if linear_col not in row or angular_col not in row:
                    raise ValueError(f'missing required columns {linear_col!r} and {angular_col!r}')
                commands.append(TraceCommand(float(row[linear_col]), float(row[angular_col])))
        if not commands:
            raise ValueError('trace CSV contained no commands')
        return cls(commands=commands, loop=loop)

    @classmethod
    def from_flexible_csv(
        cls,
        path: str | Path,
        *,
        loop: bool = True,
        max_linear_mps: float = 0.30,
        max_angular_rad_s: float = 0.75,
        joystick_deadzone: float = 0.05,
    ) -> 'TraceReplayPilot':
        path = Path(path)
        with path.open('r', encoding='utf-8', newline='') as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames:
                raise ValueError('trace CSV contained no header row')
            fields = [field.strip() for field in reader.fieldnames]
            linear_col = _first_matching_column(fields, [
                'linear_mps', 'cmd_linear_mps', 'linear_velocity_mps', 'linear_velocity', 'speed_mps', 'speed', 'v',
            ])
            angular_col = _first_matching_column(fields, [
                'angular_rad_s', 'cmd_angular_rad_s', 'angular_velocity_rad_s', 'angular_velocity', 'yaw_rate', 'omega', 'w',
            ])
            joystick_x_col = _first_matching_column(fields, [
                'joystick_x', 'axis_x', 'stick_x', 'horizontal', 'turn', 'steer', 'angular_input',
            ])
            joystick_y_col = _first_matching_column(fields, [
                'joystick_y', 'axis_y', 'stick_y', 'vertical', 'throttle', 'forward', 'linear_input',
            ])

            commands: list[TraceCommand] = []
            for row in reader:
                command = _row_to_trace_command(
                    row,
                    linear_col=linear_col,
                    angular_col=angular_col,
                    joystick_x_col=joystick_x_col,
                    joystick_y_col=joystick_y_col,
                    max_linear_mps=max_linear_mps,
                    max_angular_rad_s=max_angular_rad_s,
                    joystick_deadzone=joystick_deadzone,
                )
                if command is not None:
                    commands.append(command)
        if not commands:
            raise ValueError('could not infer any usable commands from CSV')
        return cls(commands=commands, loop=loop)

    @classmethod
    def from_wheelsim_csv(
        cls,
        path: str | Path,
        *,
        loop: bool = True,
        max_linear_mps: float = 0.30,
        max_angular_rad_s: float = 0.75,
    ) -> 'TraceReplayPilot':
        """Best-effort parser for WheelSimPhysio or similar Unity performance exports.

        This prefers explicit linear / angular columns when present. If they are absent,
        it falls back to joystick-axis style columns and scales them into velocities.
        Because public WheelSimPhysio documentation describes joystick events but does not
        document a stable CSV schema, this parser is intentionally heuristic.
        """
        return cls.from_flexible_csv(
            path,
            loop=loop,
            max_linear_mps=max_linear_mps,
            max_angular_rad_s=max_angular_rad_s,
            joystick_deadzone=0.04,
        )

    def next_command(self) -> TraceCommand:
        command = self.commands[self.index]
        if self.index + 1 < len(self.commands):
            self.index += 1
        elif self.loop:
            self.index = 0
        return command


def _first_matching_column(fieldnames: Sequence[str], candidates: Sequence[str]) -> str | None:
    normalized = {field.lower().strip(): field for field in fieldnames}
    for candidate in candidates:
        if candidate in normalized:
            return normalized[candidate]
    return None


def _safe_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _row_to_trace_command(
    row: dict[str, object],
    *,
    linear_col: str | None,
    angular_col: str | None,
    joystick_x_col: str | None,
    joystick_y_col: str | None,
    max_linear_mps: float,
    max_angular_rad_s: float,
    joystick_deadzone: float,
) -> TraceCommand | None:
    linear = _safe_float(row.get(linear_col)) if linear_col else None
    angular = _safe_float(row.get(angular_col)) if angular_col else None
    if linear is not None and angular is not None:
        return TraceCommand(linear, angular)

    jx = _safe_float(row.get(joystick_x_col)) if joystick_x_col else None
    jy = _safe_float(row.get(joystick_y_col)) if joystick_y_col else None
    if jx is None and jy is None:
        return None

    jx = 0.0 if jx is None else _normalize_axis(jx)
    jy = 0.0 if jy is None else _normalize_axis(jy)
    if abs(jx) < joystick_deadzone:
        jx = 0.0
    if abs(jy) < joystick_deadzone:
        jy = 0.0
    return TraceCommand(jy * max_linear_mps, jx * max_angular_rad_s)


def _normalize_axis(value: float) -> float:
    if abs(value) <= 1.0:
        return value
    if abs(value) <= 100.0:
        return value / 100.0
    return max(-1.0, min(1.0, value / 32767.0))


def quantize_command(linear_mps: float, angular_rad_s: float, command_library: Sequence[tuple[str, float, float]]) -> tuple[str, float, float]:
    return min(command_library, key=lambda cmd: (cmd[1] - linear_mps) ** 2 + (cmd[2] - angular_rad_s) ** 2)
