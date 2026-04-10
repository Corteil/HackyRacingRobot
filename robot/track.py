"""
robot/track.py — Load and validate a track gate definition from track.toml.

Tag numbering convention
------------------------
    Tags   0–99  : front-facing (robot approaching the gate)
    Tags 100–199  : rear-facing  (robot has passed through)

    For a clockwise circuit:
        outside post (left  when approaching) → EVEN tags
        inside  post (right when approaching) → ODD  tags

    Gate N:
        outside_front = N * 2
        outside_rear  = 100 + N * 2
        inside_front  = N * 2 + 1
        inside_rear   = 100 + N * 2 + 1

Usage
-----
    from robot.track import Track, GateDef

    track = Track.load("track.toml")

    print(track.name)           # "Test Track A"
    print(track.summary())      # "Start / Finish → Gate 1 → …"
    print(track.max_gates)      # 5

    gate = track.gate(0)
    print(gate.outside_front)   # 0
    print(gate.inside_front)    # 1
    print(gate.outside_rear)    # 100
    print(gate.inside_rear)     # 101
    print(gate.front_tags)      # (0, 1)
    print(gate.rear_tags)       # (100, 101)
    print(gate.all_tags)        # (0, 100, 1, 101)

    # Check which side a detected tag id belongs to
    gate.is_front(0)            # True
    gate.is_rear(100)           # True
    gate.is_outside(0)          # True  (even)
    gate.is_inside(1)           # True  (odd)

Python version
--------------
    Requires Python 3.11+ for the built-in tomllib.
    On Python 3.10 or earlier:  pip install tomli
"""

import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

log = logging.getLogger(__name__)

try:
    import tomllib                  # Python 3.11+
except ModuleNotFoundError:
    try:
        import tomli as tomllib     # type: ignore[no-redef]
    except ModuleNotFoundError:
        raise RuntimeError(
            "tomllib not available — on Python < 3.11 run: pip install tomli"
        )

# Tag range constants
FRONT_TAG_MIN = 0
FRONT_TAG_MAX = 99
REAR_TAG_MIN  = 100
REAR_TAG_MAX  = 199


# ── GateDef ───────────────────────────────────────────────────────────────────

@dataclass
class GateDef:
    """
    One physical gate: two posts, each with a front and rear ArUco tag.

    Naming:
        outside = left post  when approaching from the front (even tags)
        inside  = right post when approaching from the front (odd  tags)
    """
    id:            int
    label:         str
    outside_front: int      # left  post, front face  (even, 0–99)
    outside_rear:  int      # left  post, rear  face  (even, 100–199)
    inside_front:  int      # right post, front face  (odd,  0–99)
    inside_rear:   int      # right post, rear  face  (odd,  100–199)
    width_m:       float    # opening width, post centre-to-centre (metres)
    heading_hint:  float    # compass bearing hint; -1.0 = none
    notes:         str = ""

    # ── convenience accessors ─────────────────────────────────────────────────

    @property
    def front_tags(self) -> Tuple[int, int]:
        """(outside_front, inside_front) — tags the robot sees when approaching."""
        return (self.outside_front, self.inside_front)

    @property
    def rear_tags(self) -> Tuple[int, int]:
        """(outside_rear, inside_rear) — tags visible after passing through."""
        return (self.outside_rear, self.inside_rear)

    @property
    def all_tags(self) -> Tuple[int, int, int, int]:
        """All four tag ids: (outside_front, outside_rear, inside_front, inside_rear)."""
        return (self.outside_front, self.outside_rear,
                self.inside_front,  self.inside_rear)

    @property
    def has_heading_hint(self) -> bool:
        return self.heading_hint >= 0.0

    def is_front(self, tag_id: int) -> bool:
        """True if tag_id is a front-facing tag of this gate."""
        return tag_id in (self.outside_front, self.inside_front)

    def is_rear(self, tag_id: int) -> bool:
        """True if tag_id is a rear-facing tag of this gate."""
        return tag_id in (self.outside_rear, self.inside_rear)

    def is_outside(self, tag_id: int) -> bool:
        """True if tag_id belongs to the outside (left) post."""
        return tag_id in (self.outside_front, self.outside_rear)

    def is_inside(self, tag_id: int) -> bool:
        """True if tag_id belongs to the inside (right) post."""
        return tag_id in (self.inside_front, self.inside_rear)

    def _check_convention(self):
        """Warn if tag ids deviate from the expected N*2 / N*2+1 convention."""
        exp_of = self.id * 2
        exp_if = self.id * 2 + 1
        exp_or = 100 + self.id * 2
        exp_ir = 100 + self.id * 2 + 1
        if (self.outside_front, self.inside_front,
                self.outside_rear, self.inside_rear) != (exp_of, exp_if, exp_or, exp_ir):
            log.warning(
                "Gate %d (%s): tag ids %s deviate from convention "
                "(%d, %d, %d, %d).  Ensure physical tags match.",
                self.id, self.label,
                (self.outside_front, self.outside_rear,
                 self.inside_front,  self.inside_rear),
                exp_of, exp_or, exp_if, exp_ir,
            )


# ── Track ─────────────────────────────────────────────────────────────────────

@dataclass
class Track:
    """Parsed and validated track definition loaded from track.toml."""
    name:        str
    description: str
    author:      str
    version:     int
    loop:        bool
    sequence:    List[int]           # gate ids in navigation order
    gates:       Dict[int, GateDef] # gate id → GateDef

    # ── factory ──────────────────────────────────────────────────────────────

    @classmethod
    def load(cls, path: str) -> "Track":
        """Load and validate *path*.  Raises FileNotFoundError or ValueError."""
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"Track file not found: {path!r}")

        with p.open("rb") as fh:
            data = tomllib.load(fh)

        course  = data.get("course", {})
        name    = course.get("name",        "Unnamed Track")
        desc    = course.get("description", "")
        author  = course.get("author",      "")
        version = int(course.get("version", 1))
        loop    = bool(course.get("loop",   False))

        sequence = [int(g) for g in course.get("sequence", {}).get("gates", [])]
        if not sequence:
            raise ValueError("track.toml: [course.sequence] gates is empty")

        gates: Dict[int, GateDef] = {}
        for raw in data.get("gate", []):
            gid = int(raw["id"])
            gdef = GateDef(
                id            = gid,
                label         = raw.get("label",         f"Gate {gid}"),
                outside_front = int(raw.get("outside_front", gid * 2)),
                outside_rear  = int(raw.get("outside_rear",  100 + gid * 2)),
                inside_front  = int(raw.get("inside_front",  gid * 2 + 1)),
                inside_rear   = int(raw.get("inside_rear",   100 + gid * 2 + 1)),
                width_m       = float(raw.get("width_m",      1.0)),
                heading_hint  = float(raw.get("heading_hint", -1.0)),
                notes         = raw.get("notes", ""),
            )
            gdef._check_convention()
            gates[gid] = gdef

        # Every gate in sequence must have a definition
        missing = [g for g in sequence if g not in gates]
        if missing:
            raise ValueError(
                f"track.toml: sequence references undefined gate id(s): {missing}"
            )

        # All tag ids across all gates must be unique
        seen: Dict[int, int] = {}   # tag_id → gate_id
        for gid, gdef in gates.items():
            for tag_id in gdef.all_tags:
                if tag_id in seen:
                    raise ValueError(
                        f"track.toml: tag id {tag_id} is used by both "
                        f"gate {seen[tag_id]} and gate {gid}"
                    )
                seen[tag_id] = gid

        track = cls(name=name, description=desc, author=author,
                    version=version, loop=loop, sequence=sequence, gates=gates)
        log.info("Track loaded: %r  %d gate(s)  loop=%s", name, len(sequence), loop)
        return track

    # ── helpers ───────────────────────────────────────────────────────────────

    def gate(self, gate_id: int) -> GateDef:
        """Return GateDef for *gate_id*, or raise KeyError."""
        if gate_id not in self.gates:
            raise KeyError(f"No gate with id {gate_id}")
        return self.gates[gate_id]

    def gate_for_tag(self, tag_id: int) -> Optional[GateDef]:
        """Return the GateDef that owns *tag_id*, or None."""
        for gdef in self.gates.values():
            if tag_id in gdef.all_tags:
                return gdef
        return None

    @property
    def max_gates(self) -> int:
        """Number of gates in the navigation sequence."""
        return len(self.sequence)

    def next_gate_id(self, current_index: int) -> Optional[int]:
        """
        Gate id that follows *current_index* in the sequence.
        Returns None at end of sequence unless loop=True.
        """
        next_index = current_index + 1
        if next_index < len(self.sequence):
            return self.sequence[next_index]
        return self.sequence[0] if self.loop else None

    def summary(self) -> str:
        parts = " → ".join(
            self.gates[g].label for g in self.sequence if g in self.gates
        )
        return f"{self.name}{' (loop)' if self.loop else ''}: {parts}"


# ── CLI — validate and pretty-print a track file ──────────────────────────────

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "track.toml"
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    try:
        t = Track.load(path)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Track   : {t.name}")
    print(f"Desc    : {t.description}")
    print(f"Loop    : {t.loop}")
    print(f"Summary : {t.summary()}")
    print()
    print(f"{'ID':>3}  {'Label':<18}  {'Out-F':>5}  {'Out-R':>5}  "
          f"{'In-F':>5}  {'In-R':>5}  {'Width':>6}  {'Hdg':>6}  Notes")
    print("─" * 82)
    for gid in t.sequence:
        g = t.gate(gid)
        hdg = f"{g.heading_hint:.0f}°" if g.has_heading_hint else "  —"
        print(
            f"{g.id:>3}  {g.label:<18}  "
            f"{g.outside_front:>5}  {g.outside_rear:>5}  "
            f"{g.inside_front:>5}  {g.inside_rear:>5}  "
            f"{g.width_m:>5.2f}m  {hdg:>6}  {g.notes}"
        )
