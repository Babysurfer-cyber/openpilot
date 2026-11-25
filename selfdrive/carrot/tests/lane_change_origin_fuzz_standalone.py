"""Deterministic offline fuzz regression for production side-lead helper.
No native MPC or safety certification; read-only, synthetic model and radar.
"""
import importlib.util
import sys
from pathlib import Path
from random import Random
from types import SimpleNamespace

root = Path(__file__).resolve().parents[3]
source = root / "selfdrive/carrot/radar_motion/lane_change_speed_assist.py"
spec = importlib.util.spec_from_file_location("fuzz_pr63_production_helper", source)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
rng = Random(20261126)

lines = tuple(SimpleNamespace(x=(0., 100.), y=(y, y)) for y in (-5.4, -1.8, 1.8, 5.4))

def model(state, direction, physical):
  return SimpleNamespace(meta=SimpleNamespace(
    laneChangeState=state, laneChangeDirection=direction,
    laneChangeDriverRequest=physical), laneLines=lines, laneLineProbs=(.99,) * 4)

def point(direction, distance, ego, lead):
  return SimpleNamespace(source="corner235", track_id=7, d_rel=distance,
                         y_rel=3.6 if direction == "left" else -3.6,
                         v_rel=lead-ego, v_lead=lead, yv_rel=0.,
                         measured=True, radar_track_state=2)

positive_count = 0
negative_count = 0
for trial in range(7500):
  side = "left" if rng.randrange(2) == 0 else "right"
  ego = rng.uniform(18., 35.)
  dist = rng.uniform(20., 60.)
  lead = rng.uniform(2., ego - .1)
  assist = module.LaneChangeSpeedAssist()
  for i in range(6):
    pos = dist + (lead - ego) * i * .05
    phase = "off" if i < 3 else "preLaneChange"
    assert assist.update(i * .05, ego, model(phase, side, True),
                         (point(side, pos, ego, lead),)) is None
  physical = bool(trial % 4)
  candidate = assist.update(.30, ego, model("laneChangeStarting", side, physical),
                            (point(side, dist+(lead-ego)*.30, ego, lead),))
  eligible = physical and max(8., ego*.5) < dist+(lead-ego)*.30 < min(80., ego*2.5)
  # The helper must accept valid slower-lead cases; testing only the absence
  # of unsafe selections would also let an always-disabled regression pass.
  assert (candidate is not None) == eligible, (trial, ego, dist, lead, physical)
  if not physical:
    negative_count += 1
    assert candidate is None
    assert assist.update(.35, ego, model("laneChangeStarting", side, True),
                         (point(side, dist+(lead-ego)*.35, ego, lead),)) is None
  elif candidate is not None:
    positive_count += 1
    assert candidate.source == "corner235" and candidate.track_id == 7
    assert candidate.v_rel <= 0.
    assert max(8., ego * .5) < candidate.d_rel < min(80., ego * 2.5)
    assert assist.update(.35, ego, model("laneChangeStarting", side, False),
                         (point(side, dist+(lead-ego)*.35, ego, lead),)) is None
    assert assist.update(.40, ego, model("laneChangeStarting", side, True),
                         (point(side, dist+(lead-ego)*.40, ego, lead),)) is None

assert positive_count >= 5000, positive_count
assert negative_count == 1875, negative_count

for trial in range(20000):
  existing = {"status": True, "dRel": rng.uniform(12., 75.), "vLead": rng.uniform(2., 35.)}
  candidate = {"status": True, "dRel": rng.uniform(12., 75.), "vLead": rng.uniform(2., 35.)}
  actual = module.side_lead_may_replace(candidate, existing)
  expected = (candidate["dRel"] + .5 < existing["dRel"]
              and candidate["vLead"] <= existing["vLead"])
  assert actual == expected, (trial, candidate, existing)

print(f"PASS 27,500 deterministic trials: {positive_count} eligible leads selected; "
      f"{negative_count} missing-origin leads rejected; not a native MPC solve")
