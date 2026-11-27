"""Select a measured target-lane lead without changing existing radar roles.

Only identities observed in the target lane before a lane change are retained.
Published radar payloads supply current kinematics; geometry never moves an
already-started request to a different car.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
import time
from typing import Any


MAX_FRAME_AGE_S = 0.25
MAX_PAIR_SKEW_S = 0.25
MAX_STARTING_FRAME_GAP_S = 0.075
MAX_CACHED_MODELS = 8
FUTURE_TOLERANCE_NS = 1_000_000
RADAR_TO_CAMERA_M = 1.52
MIN_FORWARD_DISTANCE_M = 0.8
MIN_CAPTURE_SPEED_MPS = 1.0
MIN_LANE_WIDTH_M = 2.5
MAX_LANE_WIDTH_M = 4.8
MIN_LINE_PROBABILITY = 0.5
CONFIRMATION_FRAMES = 2
MAX_POSITION_ERROR_M = 3.0
MAX_LATERAL_ERROR_M = 1.0
MAX_SPEED_JUMP_MPS = 5.0

_KINEMATICS = ("dRel", "yRel", "vRel", "aRel", "vLead", "aLead", "vLeadK", "aLeadK", "aLeadTau", "jLead", "vLat")
_LISTS = ("leadsLeft", "leadsCenter", "leadsRight", "leadsLeft2", "leadsRight2")
_LEADS = ("leadLeft", "leadRight", "leadOne", "leadTwo")
_STATES = {"off": 0, "preLaneChange": 1, "laneChangeStarting": 2, "laneChangeFinishing": 3}
_DIRECTIONS = {"none": 0, "left": 1, "right": 2}
Identity = tuple[str, int]


def _number(value: Any) -> float | None:
  try:
    value = float(value)
    return value if math.isfinite(value) else None
  except (TypeError, ValueError, OverflowError):
    return None


def _enum(value: Any, names: dict[str, int]) -> int | None:
  raw = getattr(value, "raw", value)
  if isinstance(raw, int) and not isinstance(raw, bool):
    return raw if raw in names.values() else None
  return names.get(str(value))


def _desire(value: Any) -> int | None:
  raw = getattr(value, "raw", value)
  if isinstance(raw, int) and not isinstance(raw, bool):
    return raw if 0 <= raw <= 6 else None
  return {"none": 0, "turnLeft": 1, "turnRight": 2, "laneChangeLeft": 3,
          "laneChangeRight": 4, "keepLeft": 5, "keepRight": 6}.get(str(value))


def _identity(lead: Any) -> Identity | None:
  if not bool(getattr(lead, "status", False)) or not bool(getattr(lead, "radar", False)):
    return None
  identifier = getattr(lead, "radarTrackId", -1)
  source = str(getattr(lead, "radarSource", ""))
  if not isinstance(identifier, int) or isinstance(identifier, bool) or identifier < 0 or not source:
    return None
  return source, identifier


@dataclass(frozen=True)
class _Observation:
  payload: Any
  values: tuple[float, ...]
  timestamp_ns: int

  def follows(self, previous: _Observation) -> bool:
    dt = (self.timestamp_ns - previous.timestamp_ns) * 1e-9
    return (
      0.0 < dt <= MAX_FRAME_AGE_S
      and abs(self.values[0] - (previous.values[0] + previous.values[2] * dt)) <= MAX_POSITION_ERROR_M
      and abs(self.values[1] - previous.values[1]) <= MAX_LATERAL_ERROR_M
      and abs(self.values[6] - previous.values[6]) <= MAX_SPEED_JUMP_MPS
    )


@dataclass(frozen=True)
class _Model:
  receipt_s: float
  exposure_ns: int
  state: int | None
  direction: int | None
  desire: int | None
  lanes: tuple[tuple[tuple[float, ...], tuple[float, ...]], ...]
  probabilities: tuple[float, ...]
  horizon_m: float

  @classmethod
  def capture(cls, model: Any, receipt_s: float) -> _Model:
    meta = model.meta
    lanes = []
    for line in getattr(model, "laneLines", ()):
      xs = tuple(float(x) for x in line.x)
      ys = tuple(float(y) for y in line.y)
      if (len(xs) < 2 or len(xs) != len(ys) or not all(math.isfinite(x) for x in xs + ys)
          or any(b <= a for a, b in zip(xs, xs[1:]))):
        lanes = []
        break
      lanes.append((xs, ys))
    probabilities = tuple(float(p) for p in getattr(model, "laneLineProbs", ()))
    position = getattr(getattr(model, "position", None), "x", ())
    horizon = float(position[-1]) if len(position) else 0.0
    return cls(receipt_s, int(getattr(model, "timestampEof", 0)),
               _enum(meta.laneChangeState, _STATES), _enum(meta.laneChangeDirection, _DIRECTIONS),
               _desire(meta.desire), tuple(lanes), probabilities, horizon)

  @staticmethod
  def _interpolate(line: tuple[tuple[float, ...], tuple[float, ...]], x: float) -> float | None:
    xs, ys = line
    if not xs[0] <= x <= xs[-1]:
      return None
    index = min(len(xs) - 2, max(0, bisect_right(xs, x) - 1))
    return ys[index] + (ys[index + 1] - ys[index]) * (x - xs[index]) / (xs[index + 1] - xs[index])

  def contains(self, observation: _Observation, direction: int) -> bool:
    if len(self.lanes) != 4 or len(self.probabilities) != 4:
      return False
    outer, inner = (0, 1) if direction == 1 else (2, 3)
    if not all(math.isfinite(self.probabilities[i]) and self.probabilities[i] > MIN_LINE_PROBABILITY for i in (outer, inner)):
      return False
    x = observation.values[0] + RADAR_TO_CAMERA_M
    if not math.isfinite(self.horizon_m) or not 0.0 < x < self.horizon_m:
      return False
    left = self._interpolate(self.lanes[outer], x)
    right = self._interpolate(self.lanes[inner], x)
    return bool(left is not None and right is not None and MIN_LANE_WIDTH_M <= right - left <= MAX_LANE_WIDTH_M
                and left < -observation.values[1] < right)


class HumanLikeLaneChanges:
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._models: dict[int, _Model] = {}
    self._enabled = False
    self._blocked = False
    self._direction = 0
    self._members: set[Identity] = set()
    self._excluded: set[Identity] = set()
    self._previous: dict[Identity, _Observation] = {}
    self._confirmation: dict[int, dict[Identity, int]] = {1: {}, 2: {}}
    self._capture_model_ns = 0
    self._capture_radar_ns = 0
    self._capture_exposure_ns = 0
    self._capture_receipt_s = -math.inf
    self._last_model_ns = 0
    self._last_exposure_ns = 0
    self._last_desire: int | None = None
    self._repeat_boundary_ns = 0
    self._repeat_requested = False

  @staticmethod
  def _fresh(sm: Any, service: str, now: float) -> bool:
    try:
      receipt = _number(sm.recv_time[service])
      return bool(sm.seen[service] and sm.valid[service] and sm.alive[service]
                  and getattr(sm, "freq_ok", {}).get(service, True)
                  and receipt is not None and 0.0 <= now - receipt <= MAX_FRAME_AGE_S)
    except (AttributeError, KeyError, TypeError):
      return False

  def _clear_request(self, *, blocked: bool) -> None:
    self._blocked = blocked
    self._direction = 0
    self._members.clear()
    self._excluded.clear()
    self._previous.clear()
    self._confirmation = {1: {}, 2: {}}
    self._capture_model_ns = self._capture_radar_ns = self._capture_exposure_ns = 0
    self._capture_receipt_s = -math.inf
    self._last_exposure_ns = 0
    self._last_desire = None
    self._repeat_boundary_ns = 0
    self._repeat_requested = False

  @staticmethod
  def _observations(radar: Any, timestamp_ns: int) -> dict[Identity, _Observation]:
    observations: dict[Identity, _Observation] = {}
    ambiguous: set[Identity] = set()
    payloads = [lead for name in _LISTS for lead in getattr(radar, name, ())]
    payloads.extend(getattr(radar, name, None) for name in _LEADS)
    for lead in payloads:
      identity = _identity(lead)
      if identity is None:
        continue
      values = tuple(_number(getattr(lead, name, None)) for name in _KINEMATICS)
      if any(value is None for value in values) or values[0] <= MIN_FORWARD_DISTANCE_M:
        ambiguous.add(identity)
        continue
      observation = _Observation(lead, values, timestamp_ns)
      if identity in observations and observation.values != observations[identity].values:
        ambiguous.add(identity)
      else:
        observations[identity] = observation
    return {identity: observation for identity, observation in observations.items() if identity not in ambiguous}

  def _capture_members(self, model: _Model, model_ns: int, radar_ns: int, observations: dict[Identity, _Observation],
                       primary: Identity | None) -> None:
    if radar_ns == self._capture_radar_ns:
      return
    continuous = 0 < radar_ns - self._capture_radar_ns <= int(MAX_STARTING_FRAME_GAP_S * 1e9)
    for direction in (1, 2):
      previous = self._confirmation[direction] if continuous else {}
      self._confirmation[direction] = {
        identity: min(CONFIRMATION_FRAMES, previous.get(identity, 0) + 1)
        for identity, observation in observations.items()
        if identity != primary and identity not in self._excluded
        and observation.values[6] >= MIN_CAPTURE_SPEED_MPS and model.contains(observation, direction)
        and (identity not in self._previous or observation.follows(self._previous[identity]))
      }
    self._previous = dict(observations)
    self._capture_model_ns = model_ns
    self._capture_radar_ns = radar_ns
    self._capture_exposure_ns = model.exposure_ns
    self._capture_receipt_s = model.receipt_s

  def update(self, sm: Any, enabled: bool, *, now: float | None = None) -> Any | None:
    """Return one current native lead, or fail closed until the next request.

    Exact cached model/radar pairs confirm lane membership before STARTING.
    A held repeat opens one fresh acquisition window at its existing NONE
    pulse; the first aligned resumed desire closes that window again.
    """
    now_s = _number(time.monotonic() if now is None else now)
    if not enabled:
      self.reset()
      return None
    newly_enabled = not self._enabled
    self._enabled = True
    if now_s is None or not all(self._fresh(sm, name, now_s) for name in ("modelV2", "radarState", "carState")):
      self._clear_request(blocked=True)
      self._models.clear()
      return None

    try:
      model_ns, radar_ns = (int(sm.logMonoTime[name]) for name in ("modelV2", "radarState"))
      model = sm["modelV2"]
      radar = sm["radarState"]
      car = sm["carState"]
      current = _Model.capture(model, float(sm.recv_time["modelV2"]))
      paired_ns = int(radar.mdMonoTime)
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self._clear_request(blocked=True)
      self._models.clear()
      return None

    if model_ns > 0 and model_ns >= self._last_model_ns and current.exposure_ns > 0:
      self._models.setdefault(model_ns, current)
      self._last_model_ns = model_ns
    else:
      self._clear_request(blocked=True)
      self._models.clear()
      self._last_model_ns = 0
      return None
    self._models = {key: value for key, value in self._models.items() if 0.0 <= now_s - value.receipt_s <= MAX_FRAME_AGE_S}
    while len(self._models) > MAX_CACHED_MODELS:
      del self._models[min(self._models)]
    paired = self._models.get(paired_ns)
    valid_pair = (paired is not None and radar_ns > 0 and abs(radar_ns - paired_ns) <= int(MAX_PAIR_SKEW_S * 1e9)
                  and model_ns <= int(float(sm.recv_time["modelV2"]) * 1e9) + FUTURE_TOLERANCE_NS
                  and radar_ns <= int(float(sm.recv_time["radarState"]) * 1e9) + FUTURE_TOLERANCE_NS
                  and current.exposure_ns <= model_ns + FUTURE_TOLERANCE_NS
                  and -FUTURE_TOLERANCE_NS * 1e-9 <= now_s - current.exposure_ns * 1e-9 <= MAX_FRAME_AGE_S
                  and -FUTURE_TOLERANCE_NS * 1e-9 <= now_s - paired.exposure_ns * 1e-9 <= MAX_FRAME_AGE_S)
    v_ego = _number(getattr(car, "vEgo", None))
    torque = _number(getattr(car, "steeringTorque", None))
    override = (bool(getattr(car, "gasPressed", False)) or bool(getattr(car, "brakePressed", False))
                or (bool(getattr(car, "steeringPressed", False)) and
                    (torque is None or (current.direction == 1 and torque < 0) or (current.direction == 2 and torque > 0))))
    errors = getattr(radar, "radarErrors", None)
    radar_error = any(bool(getattr(errors, name, False)) for name in
                      ("canError", "radarFault", "wrongConfig", "radarUnavailableTemporary"))
    if (not valid_pair or not bool(getattr(car, "canValid", False)) or v_ego is None or v_ego < 0.0
        or override or radar_error or current.state not in (0, 1, 2) or current.desire in (1, 2)
        or not bool(getattr(getattr(radar, "leadOne", None), "status", False))):
      self._clear_request(blocked=current.state == 2)
      return None

    observations = self._observations(radar, radar_ns)
    primary = _identity(radar.leadOne)
    if current.state in (0, 1):
      # OFF/PRE is also the acquisition window for the next physical request.
      if self._direction or self._blocked or self._repeat_boundary_ns:
        self._clear_request(blocked=False)
      self._last_exposure_ns = current.exposure_ns
      if paired.state in (0, 1) and paired.desire not in (1, 2):
        self._capture_members(paired, paired_ns, radar_ns, observations, primary)
      return None

    requested_desire = 2 + (current.direction or 0)
    gap_ns = current.exposure_ns - self._last_exposure_ns
    new_frame = gap_ns != 0
    if (newly_enabled or self._blocked or current.direction not in (1, 2)
        or (self._direction and self._direction != current.direction)
        or self._last_exposure_ns <= 0 or gap_ns < 0
        or gap_ns > int(MAX_STARTING_FRAME_GAP_S * 1e9)
        or now_s - current.receipt_s > MAX_STARTING_FRAME_GAP_S
        or current.desire not in (0, requested_desire)):
      self._clear_request(blocked=True)
      return None

    if not self._direction:
      if current.desire != requested_desire or now_s - self._capture_receipt_s > MAX_FRAME_AGE_S:
        self._clear_request(blocked=True)
        return None
      self._direction = current.direction
      self._members = {identity for identity, count in self._confirmation[self._direction].items() if count >= CONFIRMATION_FRAMES}
    elif new_frame and self._last_desire == requested_desire and current.desire == 0:
      # Held first-detent repeats stay in STARTING and emit a NONE pulse.
      self._members.clear()
      self._confirmation = {1: {}, 2: {}}
      self._capture_model_ns = self._capture_radar_ns = self._capture_exposure_ns = 0
      self._repeat_boundary_ns = current.exposure_ns
      self._repeat_requested = False
    self._last_exposure_ns = current.exposure_ns
    self._last_desire = current.desire

    if self._repeat_boundary_ns:
      if current.desire == requested_desire:
        self._repeat_requested = True
      if paired.exposure_ns >= self._repeat_boundary_ns:
        self._capture_members(paired, paired_ns, radar_ns, observations, primary)
      if not self._repeat_requested:
        return None
      self._members = {identity for identity, count in self._confirmation[self._direction].items() if count >= CONFIRMATION_FRAMES}
      if not self._members:
        # The first aligned resumed request closes acquisition. A bad boundary
        # cannot turn into permission to acquire a different car later on.
        if paired.exposure_ns >= self._repeat_boundary_ns and paired.desire == requested_desire:
          self._clear_request(blocked=True)
        return None
      self._repeat_boundary_ns = 0

    if paired.state != 2 or paired.direction != self._direction or current.desire != requested_desire:
      return None
    for identity in self._members - self._excluded:
      observation = observations.get(identity)
      previous = self._previous.get(identity)
      if observation is None or (previous is not None and (
        (observation.timestamp_ns == previous.timestamp_ns and observation.values != previous.values)
        or (observation.timestamp_ns != previous.timestamp_ns and not observation.follows(previous))
      )):
        self._excluded.add(identity)
      elif observation is not None:
        self._previous[identity] = observation
    # Keep continuity for the next held lane as well as the current members.
    # Only current members are consumed by a disappearance during this lane.
    self._previous = dict(observations)
    candidates = [observation for identity, observation in observations.items()
                  if identity in self._members and identity not in self._excluded and identity != primary]
    selected = min(candidates, key=lambda observation: observation.values[0], default=None)
    if selected is None or _identity(selected.payload) == _identity(getattr(radar, "leadTwo", None)):
      return None
    payload = selected.payload
    return payload.as_reader() if hasattr(payload, "as_reader") else payload
