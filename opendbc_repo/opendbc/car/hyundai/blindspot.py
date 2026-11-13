"""Bridge the display-radar blind zone without changing the OEM BSD warning."""

from collections import deque
from dataclasses import dataclass

from opendbc.car.hyundai.blindspot_inputs import CORNERS, MAX_SAMPLE_AGE_NS


@dataclass
class _SideState:
  origin: str | None = None
  last_velocity: float = 0.0
  deadline_ns: int = 0
  lost: bool = False


class CornerBsdSupplement:
  def __init__(self):
    self.history = {corner: deque(maxlen=10) for corner in CORNERS}
    self.last_sample = {}
    self.velocities = dict.fromkeys(CORNERS, 0.0)
    self.sides = {side: _SideState() for side in ("L", "R")}
    self.limits = {"LF_DETECT": 6.0, "RF_DETECT": 6.0}
    self.last_time_ns = 0

  @staticmethod
  def _front_limit(velocity):
    if velocity < -0.5:
      return 6.0 + min(1.0, (-velocity - 0.5) / 4.5) * 6.0
    if velocity > 0.5:
      return 6.0 - min(1.0, (velocity - 0.5) / 4.5) * 4.0
    return 6.0

  def _update_velocity(self, corner, sample, now_ns):
    history = self.history[corner]
    previous = self.last_sample.get(corner)
    if sample is None or not 0 <= now_ns - sample.timestamp_ns <= MAX_SAMPLE_AGE_NS:
      history.clear()
      self.velocities[corner] = 0.0
      return False
    if previous is not None and sample.timestamp_ns == previous.timestamp_ns and sample.source == previous.source:
      return False
    self.last_sample[corner] = sample
    if not 0.1 < sample.distance < 20.0:
      history.clear()
      self.velocities[corner] = 0.0
      return True
    if previous is not None and (
      sample.source != previous.source or sample.timestamp_ns <= previous.timestamp_ns
      or sample.timestamp_ns - previous.timestamp_ns > MAX_SAMPLE_AGE_NS
    ):
      history.clear()
    if history and abs(sample.distance - history[-1][1]) > 1.5:
      history.clear()
    history.append((sample.timestamp_ns, sample.distance))
    # Preserve the old roughly 0.1 s motion window, using real observations
    # instead of repeated 100 Hz reads of a slower CAN message.
    while len(history) > 2 and sample.timestamp_ns - history[0][0] > 100_000_000:
      history.popleft()
    elapsed = (history[-1][0] - history[0][0]) * 1e-9
    self.velocities[corner] = (history[-1][1] - history[0][1]) / elapsed if elapsed >= 0.04 else 0.0
    return True

  def update(self, now_ns, samples):
    if now_ns < self.last_time_ns:
      self.__init__()
    self.last_time_ns = now_ns
    samples = {corner: sample for corner, sample in samples.items()
               if sample.timestamp_ns > 0 and 0 <= now_ns - sample.timestamp_ns <= MAX_SAMPLE_AGE_NS}
    fresh = {corner: self._update_velocity(corner, samples.get(corner), now_ns) for corner in CORNERS}
    warnings = {}
    for side, front, rear in (("L", "LF", "LR"), ("R", "RF", "RR")):
      state = self.sides[side]
      front_limit = self._front_limit(self.velocities[front])
      self.limits[f"{front}_DETECT"] = front_limit
      front_distance = samples[front].distance if front in samples else 0.0
      rear_distance = samples[rear].distance if rear in samples else 0.0
      seen_front = 0.1 < front_distance < front_limit
      seen_rear = 0.1 < rear_distance < 5.0
      if seen_front or seen_rear:
        active = rear if seen_rear else front
        if state.origin is None:
          state.origin = "rear" if seen_rear else "front"
        # The episode's entry direction survives LF->LR / RF->RR handoff.
        # Updating it to 'rear' here used to cancel an ego-overtake warning
        # immediately when the rear distance crossed 5 m.
        if fresh[active]:
          velocity = self.velocities[active]
          if abs(velocity) > 0.1:
            state.last_velocity = velocity
          state.deadline_ns = now_ns + int(4.5 / max(abs(velocity), 1.0) * 1e9)
        state.lost = False
        warnings[side] = True
        continue

      if not state.lost:
        if state.last_velocity != 0.0:
          hold_s = 6.0 / max(abs(state.last_velocity), 0.5) if state.origin == "front" else 3.0
          state.deadline_ns = now_ns + int(hold_s * 1e9)
          state.last_velocity = 0.0
        state.lost = True

      cancel_front = 0.1 < front_distance < 20.0 and front_distance >= front_limit and self.velocities[front] > 0.5
      cancel_rear = 0.1 < rear_distance < 20.0 and rear_distance >= 5.0 and self.velocities[rear] > 0.5
      if cancel_front or (state.origin != "front" and cancel_rear):
        state.deadline_ns = 0
      warnings[side] = now_ns < state.deadline_ns
      if not warnings[side]:
        self.sides[side] = _SideState()
    return warnings
