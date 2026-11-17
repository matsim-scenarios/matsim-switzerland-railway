"""Input/output paths of the table builders.

`prepare/build_tables.py` calls :func:`configure` once, then imports the table modules, which read
the paths from here at import time.  Keeping the paths in one mutable module (instead of module
constants in every builder) is what lets the same code write into `matsim_input/tables/` or into a
scratch directory for reproducibility checks.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Paths:
    """Absolute paths of every input and output of the pipeline."""

    root: Path          # repository root
    osm: Path           # original_data/osm/switzerland_railways.osm  (railway=* only)
    pbf: Path           # original_data/osm/switzerland-latest.osm.pbf (full extract, with relations)
    schedule: Path      # matsim_input/tmp/schedule_gtfs_filtered.xml.gz (unmapped, whole Switzerland)
    network: Path       # matsim_input/tmp/network_osm.xml.gz (pt2matsim network)
    out: Path           # output directory; one sub-directory per table

    def table(self, name: str) -> Path:
        """Output directory of one table (`A01`, `A05`, `DIR`, ...), created on demand."""
        p = self.out / name
        p.mkdir(parents=True, exist_ok=True)
        return p


PATHS: Paths | None = None


def configure(root: Path | None = None, osm: Path | None = None, pbf: Path | None = None,
              schedule: Path | None = None, network: Path | None = None,
              out: Path | None = None) -> Paths:
    """Set the global paths.  Defaults are repository-relative."""
    global PATHS
    r = Path(root or ROOT).resolve()
    PATHS = Paths(
        root=r,
        osm=Path(osm).resolve() if osm else r / "original_data/osm/switzerland_railways.osm",
        pbf=Path(pbf).resolve() if pbf else r / "original_data/osm/switzerland-latest.osm.pbf",
        schedule=Path(schedule).resolve() if schedule else r / "matsim_input/tmp/schedule_gtfs_filtered.xml.gz",
        network=Path(network).resolve() if network else r / "matsim_input/tmp/network_osm.xml.gz",
        out=Path(out).resolve() if out else r / "matsim_input/tables",
    )
    return PATHS


def get() -> Paths:
    if PATHS is None:
        raise RuntimeError("prepare.paths.configure() must be called before importing a table builder")
    return PATHS
