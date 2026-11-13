"""Suspend signal-specific control only on a fresh, explicit vNavi road class."""
import time
import math

NAVI_TIMEOUT = 2.0
ROAD_TIMEOUT = 10.0  # v2 JSON stream's advertised stale_timeout_ms


def _vnavi_signal_detection_suspended(sm, now=None):
  now = time.monotonic() if now is None else now
  try:
    if not (sm.seen['carrotNavi'] and sm.valid['carrotNavi'] and sm.alive['carrotNavi']):
      return False
    if not 0 <= now - sm.recv_time['carrotNavi'] <= NAVI_TIMEOUT:
      return False
    navi = sm['carrotNavi']
    lane = navi.laneCurrent
    if navi.schemaVersion != 1 or not navi.connected or not navi.sessionId:
      return False
    if navi.navigationStatus.meta.present and navi.navigationStatus.offRoute:
      return False
    # roadCategory defaults to zero in old messages and when the source omits it.
    # Neither case is evidence of a motorway.
    return bool(lane.meta.present and lane.roadCategoryValid and lane.roadCategory in (0, 1)
                and lane.meta.receivedMonoTimeNanos > 0
                and 0 <= now - lane.meta.receivedMonoTimeNanos / 1e9 <= ROAD_TIMEOUT)
  except (AttributeError, KeyError, TypeError):
    return False


def road_category_observation(payload, timestamp_ns):
  """Keep explicit legacy navigation evidence separate from roadcate's default 0."""
  value = payload.get('roadcate')
  try:
    category = int(value)
    valid = (not isinstance(value, bool) and math.isfinite(float(value))
             and float(value) == category and 0 <= category <= 8 and timestamp_ns > 0)
    return (category, True, timestamp_ns) if valid else (0, False, 0)
  except (TypeError, ValueError, OverflowError):
    return 0, False, 0


def signal_detection_suspended(sm, now=None):
  now = time.monotonic() if now is None else now
  if _vnavi_signal_detection_suspended(sm, now):
    return True
  try:
    service = 'carrotMan'
    if not (sm.seen[service] and sm.valid[service] and sm.alive[service]
            and 0 <= now - sm.recv_time[service] <= NAVI_TIMEOUT):
      return False
    navi = sm[service]
    return bool(navi.activeCarrot > 1 and navi.roadCategoryValid
                and navi.roadCategory in (0, 1) and navi.roadCategoryMonoTimeNanos > 0
                and 0 <= now - navi.roadCategoryMonoTimeNanos / 1e9 <= ROAD_TIMEOUT)
  except (AttributeError, KeyError, TypeError, ValueError):
    return False
