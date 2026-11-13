"""Recognize the deliberately disabled EV6 test's temporary Panda parameter.

This only accounts for an expected configuration difference. Panda health,
controls-allowed mismatches and faults are still checked by selfdrived.
"""

TEST_PARAM = 1024
CANFD_CAMERA_FLAGS = (1 << 13) | (1 << 3)
EV_CAMERA_PARAMS = 1 | 8


def _eligible(cp, panda, enabled):
  if (enabled or cp.brand != "hyundai" or cp.carFingerprint != "KIA_EV6"
      or cp.passive or cp.dashcamOnly or len(cp.safetyConfigs) != 1
      or cp.flags & CANFD_CAMERA_FLAGS != CANFD_CAMERA_FLAGS or cp.flags & (1 << 27)):
    return False
  original = cp.safetyConfigs[0]
  return (str(original.safetyModel) == "hyundaiCanfd"
          and original.safetyParam & EV_CAMERA_PARAMS == EV_CAMERA_PARAMS
          and not original.safetyParam & TEST_PARAM
          and panda.safetyModel == original.safetyModel
          and panda.safetyParam == original.safetyParam | TEST_PARAM
          and panda.alternativeExperience == cp.alternativeExperience
          and not panda.controlsAllowed)


def _token(value):
  if isinstance(value, bytes):
    try:
      value = value.decode("ascii")
    except UnicodeDecodeError:
      return None
  if not isinstance(value, str) or value.count(":") != 1:
    return None
  mode, nonce = value.split(":")
  if mode not in ("test", "normal") or not 1 <= len(nonce) <= 64:
    return None
  if not all("a" <= c <= "z" or "A" <= c <= "Z" or "0" <= c <= "9" or c in "-_" for c in nonce):
    return None
  return mode, nonce


def correlated_blinker_request(request, ack):
  request, ack = _token(request), _token(ack)
  return (request is not None and ack is not None and request[1] == ack[1]
          and (request == ack or request[0] == "normal" and ack[0] == "test"))


class BlinkerSafetyExpectation:
  def __init__(self):
    self.last_read = None
    self.request = self.ack = None

  def matches(self, cp, panda, enabled, params, now):
    # The usual driving path never reads the test request files. An enabled
    # controller or any other safety difference also keeps normal error checks.
    if not _eligible(cp, panda, enabled):
      return False
    if self.last_read is None or now < self.last_read or now - self.last_read >= 0.1:
      self.request = params.get("HyundaiBlinkerTestSafetyRequest")
      self.ack = params.get("HyundaiBlinkerTestSafetyAck")
      self.last_read = now
    return correlated_blinker_request(self.request, self.ack)
