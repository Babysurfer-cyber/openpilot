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


def atc_skip_early_window_m(guide_m, cruise_kph, turn_speed_kph, atc_dist_m, decel_mps2):
  """Give a pre-deceleration skip opportunity even when ATC slows before the HUD guide.

  Uses the same v² = vf² + 2*a*x deceleration model and 2s target-time
  buffer as CarrotServ.update_auto_turn. Never changes the ATC decel policy.
  """
  try:
    values = tuple(float(v) for v in
                   (guide_m, cruise_kph, turn_speed_kph, atc_dist_m, decel_mps2))
  except (TypeError, ValueError, OverflowError):
    return 0.0
  if not all(math.isfinite(v) for v in values):
    return 0.0
  guide_m, cruise_kph, turn_speed_kph, atc_dist_m, decel_mps2 = values
  if guide_m <= 0:
    return 0.0
  if cruise_kph <= turn_speed_kph or turn_speed_kph <= 0 or decel_mps2 <= 0:
    return guide_m

  cruise_mps, turn_mps = cruise_kph / 3.6, turn_speed_kph / 3.6
  braking_m = (cruise_mps ** 2 - turn_mps ** 2) / (2.0 * decel_mps2)
  # Four seconds to request a skip BEFORE the calculated braking onset;
  # the remaining ATC control itself is untouched.
  early_m = braking_m + max(0.0, atc_dist_m) + turn_mps * 2.0 + min(160.0, max(60.0, cruise_mps * 4.0))
  return max(guide_m, min(1000.0, early_m))


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

    CarrotNavi wire sequences track packet revisions, not turn identity.
    Use the session, turn type, descriptive text and optional turn-point
    coordinates to retain a cancellation across packet updates. Without a
    point (including legacy TBT), a large distance increase is a fallback
    hint that the next maneuver has started.
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

    # CarrotNavi's wire sequence is a packet update counter, not a maneuver
    # ID. A valid navigation point is the strongest available identity.
    # In the absence of a point, retain the skip across text-identical
    # guidance updates, then use a distance jump as a new-turn heuristic.
    same_maneuver = self.guidance_key == guidance_key
    same_position = False
    if (isinstance(self.guidance_key, tuple) and isinstance(guidance_key, tuple)
        and len(self.guidance_key) == len(guidance_key) == 7
        and self.guidance_key[0] == guidance_key[0] == "carrotNavi"
        and self.guidance_key[1:3] == guidance_key[1:3]):
      old_point, new_point = self.guidance_key[6], guidance_key[6]
      if old_point is not None and new_point is not None:
        same_position = old_point == new_point
        same_maneuver = same_position
      else:
        same_maneuver = self.guidance_key[3:6] == guidance_key[3:6]

    no_confirmed_position = not same_position
    distance_jump = (self.last_distance_m is not None and
                     distance_m > self.last_distance_m + ATC_GUIDANCE_RESET_JUMP_M)
    if not same_maneuver or (no_confirmed_position and distance_jump):
      self.skipped = False

    self.guidance_key = guidance_key
    self.last_distance_m = distance_m
    return self.skipped

  def request(self, *, enabled, cruise_active, turn_type, distance_m, guide_distance_m,
              atc_type, deceleration_started):
    """Only cancel a current turn/fork before deceleration in the eligible pre-braking window."""
    if self.skipped or not enabled or not cruise_active or turn_type not in ATC_ONESHOT_TURNS:
      return False
    if (not math.isfinite(distance_m) or not math.isfinite(guide_distance_m) or
        distance_m <= 0 or guide_distance_m <= 0 or distance_m > guide_distance_m):
      return False
    if deceleration_started or not atc_type or atc_type.endswith(" canceled") or atc_type == "none":
      return False
    self.skipped = True
    return True
