from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence


class TickWriter:
    def write_tick(
        self, record: Mapping[str, Any]
    ) -> None:  # pragma: no cover - protocol-like
        raise NotImplementedError


class CsvTickWriter(TickWriter):
    def __init__(self, path: Path, header: Optional[Sequence[str]] = None) -> None:
        self.path = path
        self._fh = path.open("w", newline="", encoding="utf-8")
        self._header = (
            list(header)
            if header
            else [
                "t",
                "x",
                "y",
                "psi",
                "u",
                "v",
                "r",
                "rpm",
                "rudder_angle",
                "wind_speed",
                "wind_dir_from",
                "current_speed",
                "current_dir_to",
                "X",
                "Y",
                "N",
            ]
        )
        self._writer = csv.writer(self._fh)
        self._writer.writerow(self._header)

    def write_tick(self, record: Mapping[str, Any]) -> None:
        row = [record.get(k, 0) for k in self._header]
        self._writer.writerow(row)

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


class JsonlTickWriter(TickWriter):
    def __init__(self, path: Path) -> None:
        self._fh = path.open("w", encoding="utf-8")

    def write_tick(self, record: Mapping[str, Any]) -> None:
        self._fh.write(json.dumps(record) + "\n")

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


class SummaryWriter:
    def write_summary(
        self, summary: Mapping[str, Any]
    ) -> None:  # pragma: no cover - protocol-like
        raise NotImplementedError


class SummaryJsonWriter(SummaryWriter):
    def __init__(self, path: Path) -> None:
        self.path = path

    def write_summary(self, summary: Mapping[str, Any]) -> None:
        self.path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
