"""Fresh OEM display-radar observations, before cluster message forwarding."""

from dataclasses import dataclass
import math


CORNER_MESSAGES = ("ADRV_0x1ea", "CCNC_0x162")
STOCK_BSD_MESSAGE = "BLINDSPOTS_REAR_CORNERS"
MAX_SAMPLE_AGE_NS = 300_000_000
CORNERS = ("LF", "RF", "LR", "RR")


@dataclass(frozen=True)
class CornerSample:
  distance: float
  lateral: float
  timestamp_ns: int
  source: str


def register_seen_messages(parser, names):
  """Optional messages can first appear after the startup discovery window."""
  for name in names:
    message = parser.dbc.name_to_msg.get(name)
    if message is not None and message.address in parser.seen_addresses and message.address not in parser.addresses:
      parser._add_message(name, math.nan)
    if message is not None and message.address in parser.message_states:
      parser.message_states[message.address].ignore_alive = True


def _fresh(timestamp_ns, now_ns):
  return timestamp_ns > 0 and 0 <= now_ns - timestamp_ns <= MAX_SAMPLE_AGE_NS


def read_corner_samples(parser, now_ns):
  # Use the camera-side OEM stream. The ECAN copy can be transmitted by our
  # own cluster forwarding code, so it must not feed back into BSD detection.
  result = {}
  for name in CORNER_MESSAGES:
    timestamps = parser.ts_nanos.get(name, {})
    values = parser.vl.get(name, {})
    for corner in CORNERS:
      distance_signal = f"{corner}_DETECT_DISTANCE"
      timestamp = timestamps.get(distance_signal, 0)
      if not _fresh(timestamp, now_ns):
        continue
      distance = float(values.get(distance_signal, 0.0))
      lateral = float(values.get(f"{corner}_DETECT_LATERAL", 0.0))
      if not math.isfinite(distance) or not math.isfinite(lateral):
        distance, lateral = 0.0, 0.0
      # Keep a timestamped empty observation when the distance disappears.
      # Detection-class codes differ between these messages; do not infer a
      # common validity bit. The BSD warning area is checked separately.
      if distance <= 0.1:
        distance, lateral = 0.0, 0.0
      sample = CornerSample(distance, lateral, timestamp, name)
      previous = result.get(corner)
      if previous is None or (
        (sample.distance <= 0.1, sample.distance, -sample.timestamp_ns)
        < (previous.distance <= 0.1, previous.distance, -previous.timestamp_ns)
      ):
        # A valid close object cannot be hidden by a farther/stale stream.
        # Its longitudinal and lateral distances always come from one source.
        result[corner] = sample
  return result


def read_stock_blindspots(parsers, now_ns):
  left = right = False
  for parser in parsers:
    timestamps = parser.ts_nanos.get(STOCK_BSD_MESSAGE, {})
    if not _fresh(timestamps.get("FL_INDICATOR", 0), now_ns):
      continue
    values = parser.vl[STOCK_BSD_MESSAGE]
    left |= any(values.get(name, 0) > 0 for name in ("FL_INDICATOR", "INDICATOR_LEFT_TWO", "INDICATOR_LEFT_FOUR"))
    right |= any(values.get(name, 0) > 0 for name in ("FR_INDICATOR", "INDICATOR_RIGHT_TWO", "INDICATOR_RIGHT_FOUR"))
  return left, right
