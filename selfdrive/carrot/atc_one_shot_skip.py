"""One-shot ATC maneuver skip state. No CAN output or cruise state changes."""

import math

ATC_ONESHOT_TURNS = frozenset((1, 2, 3, 4))
ATC_SKIP_REQUEST_MAX_AGE_NS = 1_000_000_000
ATC_GUIDANCE_RESET_JUMP_M = 70


def fresh_atc_skip_request(raw, previous_raw, now_ns):
  """A release event is consumed only once and is never replayed after restart."""
  if not raw or raw == previous_raw:
    return False
  try:
    sent_ns = int(raw)
  except (TypeError, ValueError):
    return False
  return 0 <= now_ns - sent_ns <= ATC_SKIP_REQUEST_MAX_AGE_NS


def should_request_atc_skip(*, paddle_mode, button_type, left_paddle_type,
                            long_pressed, cruise_enabled, cruise_available):
  """Only a short left-paddle release in mode 4 may request an ATC one-shot skip."""
  return (paddle_mode == 4 and button_type == left_paddle_type and
          not long_pressed and cruise_enabled and cruise_available)


class AtcOneShotSkip:
  def __init__(self):
    self.guidance_key = None
    self.last_distance_m = None
    self.skipped = False

  def reset(self):
    self.guidance_key = None
    self.last_distance_m = None
    self.skipped = False

  def observe(self, guidance_key, turn_type, distance_m):
    """Keep a skip latched for the exact maneuver, not the full navigation route.

    CarrotNavi uses a session+guidance sequence. Legacy TBT falls back to
    type/text and a large distance jump to identify the next maneuver.
    """
    if guidance_key is None or turn_type not in ATC_ONESHOT_TURNS or not math.isfinite(distance_m):
      # Temporary navigation absence must not re-arm the very same turn
      # when its original message returns. Retain the identity and last
      # known distance until the next real maneuver is observed.
      return False
    if distance_m <= -50:
      # The previously targeted maneuver has been passed.
      self.reset()
      return False

    if (self.guidance_key != guidance_key or
        (self.last_distance_m is not None and distance_m > self.last_distance_m + ATC_GUIDANCE_RESET_JUMP_M)):
      self.skipped = False

    self.guidance_key = guidance_key
    self.last_distance_m = distance_m
    return self.skipped

  def request(self, *, enabled, cruise_active, turn_type, distance_m, guide_distance_m,
              atc_type, deceleration_started):
    """Only cancel a not-yet-decelerating current turn/fork in its guide window."""
    if self.skipped or not enabled or not cruise_active or turn_type not in ATC_ONESHOT_TURNS:
      return False
    if (not math.isfinite(distance_m) or not math.isfinite(guide_distance_m) or
        distance_m <= 0 or guide_distance_m <= 0 or distance_m > guide_distance_m):
      return False
    if deceleration_started or not atc_type or atc_type.endswith((" prepare", " canceled")) or atc_type == "none":
      return False
    self.skipped = True
    return True
