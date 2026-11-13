#pragma once

// Debug-only, parked EV6 experiment. This mode never permits steering or ACC TX.
// UDS restoration remains available if an input disappears during the experiment.
const uint16_t HYUNDAI_PARAM_BLINKER_TEST = 1024U;
bool hyundai_canfd_blinker_test = false;
bool hyundai_canfd_blinker_test_authorized = false;

const CanMsg HYUNDAI_BLINKER_TEST_TX_MSGS[] = {
  {0x7B1, 0, 8}, {0x165, 0, 24}, {0x16A, 0, 32}, {0x1EA, 0, 32},
};
// Keep ordinary safety health checks for the high-rate parked inputs. The
// experiment additionally validates these, stalks, lamps and source frames below.
RxCheck hyundai_blinker_test_rx_checks[] = {
  {.msg = {{0x35, 0, 32, .max_counter = 255U, .frequency = 100U}}},
  {.msg = {{0xA0, 0, 24, .max_counter = 255U, .frequency = 100U}}},
  {.msg = {{0x175, 0, 24, .max_counter = 255U, .frequency = 50U}}},
};

typedef enum {
  HYUNDAI_BLINKER_IDLE, HYUNDAI_BLINKER_EXT_PENDING, HYUNDAI_BLINKER_EXTENDED,
  HYUNDAI_BLINKER_DISABLE_PENDING, HYUNDAI_BLINKER_DISABLED, HYUNDAI_BLINKER_PULSE,
  HYUNDAI_BLINKER_RESTORE_PENDING, HYUNDAI_BLINKER_RESTORED,
  HYUNDAI_BLINKER_DEFAULT_PENDING, HYUNDAI_BLINKER_WAIT_STOCK, HYUNDAI_BLINKER_HOLD,
} HyundaiBlinkerTestPhase;

typedef struct {
  bool seen;
  bool valid;
  bool captured;
  uint8_t counter;
  uint32_t ts;
  CANPacket_t packet;
} HyundaiBlinkerTestSource;

static struct {
  HyundaiBlinkerTestPhase phase;
  HyundaiBlinkerTestSource source[8];  // accelerator, wheels, brake, stalk, lamps, SPAS1/2, ADRV
  bool parked[3];
  bool neutral_stalk;
  bool lamps_off;
  bool aborted;
  bool restore_ack;
  bool lamp_feedback;
  bool stock_resumed[2];
  bool spas_sent[2];
  uint8_t spas_counter[2];
  uint32_t spas_tx_ts[2];
  bool hold_sent;
  uint32_t hold_tx_ts;
  CANPacket_t adrv_history[3];
  uint32_t adrv_history_ts[3];
  bool adrv_history_valid[3];
  uint8_t direction;
  uint32_t session_ts;
  uint32_t disabled_ts;
  uint32_t pulse_ts;
} hyundai_blinker_test_state;

static bool hyundai_blinker_test_crc(const CANPacket_t *p) {
  return hyundai_canfd_get_checksum(p) == hyundai_common_canfd_compute_checksum(p);
}

static bool hyundai_blinker_test_fresh(int index, uint32_t now, uint32_t age) {
  const HyundaiBlinkerTestSource *s = &hyundai_blinker_test_state.source[index];
  return s->valid && (get_ts_elapsed(now, s->ts) <= age);
}

static bool hyundai_blinker_test_guard(uint32_t now) {
  bool valid = hyundai_blinker_test_state.neutral_stalk;
  for (int i = 0; i < 3; i++) {
    valid = valid && hyundai_blinker_test_state.parked[i] && hyundai_blinker_test_fresh(i, now, 100000U);
  }
  valid = valid && hyundai_blinker_test_fresh(3, now, 300000U) && hyundai_blinker_test_fresh(4, now, 300000U);
  return valid;
}

static bool hyundai_blinker_test_running(uint32_t now) {
  bool valid = hyundai_blinker_test_guard(now) && !hyundai_blinker_test_state.aborted;
  if ((hyundai_blinker_test_state.phase != HYUNDAI_BLINKER_IDLE) &&
      (hyundai_blinker_test_state.phase != HYUNDAI_BLINKER_HOLD)) {
    valid = valid && (get_ts_elapsed(now, hyundai_blinker_test_state.session_ts) <= 4000000U);
  }
  if (hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_HOLD) {
    valid = valid && hyundai_blinker_test_state.restore_ack &&
            hyundai_blinker_test_fresh(5, now, 100000U) && hyundai_blinker_test_fresh(6, now, 100000U) &&
            ((GET_BYTE(&hyundai_blinker_test_state.source[6].packet, 16) & 0xE0U) == 0U);
  }
  if (!valid && (hyundai_blinker_test_state.phase != HYUNDAI_BLINKER_IDLE)) {
    hyundai_blinker_test_state.aborted = true;
  }
  return valid;
}

static void hyundai_blinker_test_rx(const CANPacket_t *p) {
  controls_allowed = false;
  const int addr = GET_ADDR(p);
  const int bus = GET_BUS(p);
  const int len = GET_LEN(p);
  const uint32_t now = microsecond_timer_get();
  int index = -1;
  if (bus == 0) {
    if ((addr == 0x35) && (len == 32)) index = 0;
    if ((addr == 0xA0) && (len == 24)) index = 1;
    if ((addr == 0x175) && (len == 24)) index = 2;
    if ((addr == 0x3C1) && (len == 8)) index = 3;
    if ((addr == 0x413) && (len == 8)) index = 4;
    if ((addr == 0x165) && (len == 24)) index = 5;
    if ((addr == 0x16A) && (len == 32)) index = 6;
  }
  if ((bus == 2) && (addr == 0x1EA) && (len == 32)) index = 7;

  if (index >= 0) {
    HyundaiBlinkerTestSource *s = &hyundai_blinker_test_state.source[index];
    const bool alt_counter = (index == 3) || (index == 4);
    const uint8_t counter = alt_counter ? (GET_BYTE(p, 1) >> 4) : GET_BYTE(p, 2);
    const uint8_t mask = alt_counter ? 15U : 255U;
    // The checksum algorithm of the two 8-byte body messages is not known.
    // Require a progressing counter and short freshness; do not call CRC16 on them.
    const bool checksum = alt_counter || hyundai_blinker_test_crc(p);
    const uint8_t delta = (counter - s->counter) & mask;
    const bool advancing = s->seen && ((delta == 1U) || (delta == 2U));
    s->valid = checksum && advancing;
    s->seen = checksum;
    s->counter = counter;
    if (s->valid) {
      s->captured = true;
      s->ts = now;
      s->packet = *p;
      if (index == 7) {
        for (int i = 2; i > 0; i--) {
          hyundai_blinker_test_state.adrv_history[i] = hyundai_blinker_test_state.adrv_history[i - 1];
          hyundai_blinker_test_state.adrv_history_ts[i] = hyundai_blinker_test_state.adrv_history_ts[i - 1];
          hyundai_blinker_test_state.adrv_history_valid[i] = hyundai_blinker_test_state.adrv_history_valid[i - 1];
        }
        hyundai_blinker_test_state.adrv_history[0] = *p;
        hyundai_blinker_test_state.adrv_history_ts[0] = now;
        hyundai_blinker_test_state.adrv_history_valid[0] = true;
      }
      if (index == 0) hyundai_blinker_test_state.parked[0] = ((GET_BYTE(p, 24) & 7U) == 0U) && (GET_BYTE(p, 5) == 0U);
      if (index == 1) {
        bool zero = true;
        for (int b = 8; b < 16; b++) zero = zero && (GET_BYTE(p, b) == 0U);
        hyundai_blinker_test_state.parked[1] = zero;
      }
      if (index == 2) hyundai_blinker_test_state.parked[2] = GET_BIT(p, 81U) && (((GET_BYTE(p, 10) >> 6) & 3U) == 1U);
      if (index == 3) hyundai_blinker_test_state.neutral_stalk = !GET_BIT(p, 30U) && !GET_BIT(p, 32U) && !GET_BIT(p, 36U) && !GET_BIT(p, 38U);
      if (index == 4) {
        const bool left = GET_BIT(p, 20U) || GET_BIT(p, 59U);
        const bool right = GET_BIT(p, 22U) || GET_BIT(p, 61U);
        hyundai_blinker_test_state.lamps_off = !left && !right;
        if (hyundai_blinker_test_state.phase != HYUNDAI_BLINKER_IDLE) {
          const bool unexpected = (left && (hyundai_blinker_test_state.direction != 3U)) ||
                                  (right && (hyundai_blinker_test_state.direction != 4U));
          if (unexpected) hyundai_blinker_test_state.aborted = true;
          if ((left && (hyundai_blinker_test_state.direction == 3U)) ||
              (right && (hyundai_blinker_test_state.direction == 4U))) hyundai_blinker_test_state.lamp_feedback = true;
        }
      }
      if ((index == 5) || (index == 6)) {
        if ((index == 6) && (hyundai_blinker_test_state.phase != HYUNDAI_BLINKER_IDLE) && ((GET_BYTE(p, 16) & 0xE0U) != 0U)) {
          hyundai_blinker_test_state.aborted = true;
        }
        if (hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_PULSE) hyundai_blinker_test_state.aborted = true;
        if (hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_WAIT_STOCK) {
          if ((index == 6) && ((GET_BYTE(p, 16) & 0xE0U) != 0U)) hyundai_blinker_test_state.aborted = true;
          hyundai_blinker_test_state.stock_resumed[index - 5] = true;
          if (hyundai_blinker_test_state.stock_resumed[0] && hyundai_blinker_test_state.stock_resumed[1]) {
            const bool handoff = hyundai_blinker_test_state.restore_ack && hyundai_blinker_test_state.lamp_feedback &&
                                 (get_ts_elapsed(now, hyundai_blinker_test_state.session_ts) <= 4000000U);
            if (!handoff) hyundai_blinker_test_state.aborted = true;
            hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_HOLD;
          }
        }
      }
    }
    (void)hyundai_blinker_test_running(now);
  }

  if ((bus == 0) && (addr == 0x7B9) && (len == 8) && (GET_BYTE(p, 0) >= 2U) && (GET_BYTE(p, 0) <= 7U)) {
    const uint8_t service = GET_BYTE(p, 1);
    const uint8_t sub = GET_BYTE(p, 2);
    if ((hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_EXT_PENDING) && (service == 0x50U) && (sub == 3U)) {
      hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_EXTENDED;
    } else if ((hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_DISABLE_PENDING) && (service == 0x68U) && (sub == 1U)) {
      hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_DISABLED;
      hyundai_blinker_test_state.disabled_ts = now;
    } else if ((hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_RESTORE_PENDING) && (service == 0x68U) && (sub == 0U)) {
      hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_RESTORED;
      hyundai_blinker_test_state.restore_ack = true;
    } else if ((hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_DEFAULT_PENDING) && (service == 0x50U) && (sub == 1U)) {
      hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_WAIT_STOCK;
      hyundai_blinker_test_state.stock_resumed[0] = false;
      hyundai_blinker_test_state.stock_resumed[1] = false;
      if (!hyundai_blinker_test_state.restore_ack) hyundai_blinker_test_state.aborted = true;
    }
  }
}

static bool hyundai_blinker_test_same_body(const CANPacket_t *p, int index, bool spas2) {
  const HyundaiBlinkerTestSource *s = &hyundai_blinker_test_state.source[index];
  // Recovery uses the last verified snapshot even if a later corrupt frame
  // invalidated freshness. Nonzero commands separately require live guards.
  bool same = s->captured;
  for (int i = 3; i < GET_LEN(p); i++) {
    uint8_t mask = 255U;
    if (spas2 && (i == 16)) mask = 0x1FU;
    if ((index == 7) && (i == 3)) mask = 0x9FU;
    same = same && ((GET_BYTE(p, i) & mask) == (GET_BYTE(&s->packet, i) & mask));
  }
  return same && hyundai_blinker_test_crc(p);
}

static bool hyundai_blinker_test_adrv_command(const CANPacket_t *p, uint32_t now, bool off) {
  bool matched = false;
  for (int i = 0; i < 3; i++) {
    const CANPacket_t *stock = &hyundai_blinker_test_state.adrv_history[i];
    bool same = hyundai_blinker_test_state.adrv_history_valid[i] &&
                (off || (get_ts_elapsed(now, hyundai_blinker_test_state.adrv_history_ts[i]) <= 100000U));
    for (int b = 2; b < 32; b++) {
      const uint8_t mask = (b == 3) ? 0x9FU : 255U;
      same = same && ((GET_BYTE(p, b) & mask) == (GET_BYTE(stock, b) & mask));
    }
    matched = matched || same;
  }
  return matched && hyundai_blinker_test_crc(p);
}

static bool hyundai_blinker_test_tx(const CANPacket_t *p) {
  controls_allowed = false;
  if (!hyundai_canfd_blinker_test_authorized) return false;
  const uint32_t now = microsecond_timer_get();
  const int addr = GET_ADDR(p);
  const bool running = hyundai_blinker_test_running(now);
  bool tx = false;
  if ((GET_BUS(p) != 0) || !((addr == 0x7B1) || (addr == 0x165) || (addr == 0x16A) || (addr == 0x1EA))) return false;

  if ((addr == 0x7B1) && (GET_LEN(p) == 8)) {
    bool padding = true;
    for (int i = 4; i < 8; i++) padding = padding && (GET_BYTE(p, i) == 0U);
    const bool session = (GET_BYTE(p, 0) == 2U) && (GET_BYTE(p, 1) == 0x10U) && (GET_BYTE(p, 3) == 0U) && padding;
    const bool communication = (GET_BYTE(p, 0) == 3U) && (GET_BYTE(p, 1) == 0x28U) && (GET_BYTE(p, 3) == 1U) && padding;
    if (communication && (GET_BYTE(p, 2) == 0U)) {
      tx = true;  // Explicit recovery must survive stale vehicle data or a timeout.
      hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_RESTORE_PENDING;
    } else if (session && (GET_BYTE(p, 2) == 1U)) {
      tx = true;
      hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_DEFAULT_PENDING;
    } else if (session && (GET_BYTE(p, 2) == 3U)) {
      const bool idle = hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_IDLE;
      tx = idle && hyundai_blinker_test_guard(now) && hyundai_blinker_test_state.lamps_off &&
           hyundai_blinker_test_fresh(5, now, 100000U) && hyundai_blinker_test_fresh(6, now, 100000U) &&
           ((GET_BYTE(&hyundai_blinker_test_state.source[6].packet, 16) & 0xE0U) == 0U);
      if (tx) {
        hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_EXT_PENDING;
        hyundai_blinker_test_state.session_ts = now;
        hyundai_blinker_test_state.aborted = false;
        hyundai_blinker_test_state.restore_ack = false;
        hyundai_blinker_test_state.lamp_feedback = false;
        hyundai_blinker_test_state.direction = 0U;
        hyundai_blinker_test_state.spas_sent[0] = false;
        hyundai_blinker_test_state.spas_sent[1] = false;
      }
    } else if (communication && (GET_BYTE(p, 2) == 1U)) {
      tx = running && (hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_EXTENDED);
      if (tx) hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_DISABLE_PENDING;
    }
  }

  if (((addr == 0x165) && (GET_LEN(p) == 24)) || ((addr == 0x16A) && (GET_LEN(p) == 32))) {
    const int i = (addr == 0x165) ? 0 : 1;
    const uint8_t direction = (i == 1) ? (GET_BYTE(p, 16) >> 5) : 0U;
    const bool off = (i == 1) && (direction == 0U);
    const bool rate = !hyundai_blinker_test_state.spas_sent[i] || (get_ts_elapsed(now, hyundai_blinker_test_state.spas_tx_ts[i]) >= 20000U);
    const uint8_t prev = hyundai_blinker_test_state.spas_sent[i] ? hyundai_blinker_test_state.spas_counter[i] : GET_BYTE(&hyundai_blinker_test_state.source[5 + i].packet, 2);
    const bool counter = GET_BYTE(p, 2) == (uint8_t)(prev + 1U);
    const bool quiet = (get_ts_elapsed(now, hyundai_blinker_test_state.disabled_ts) >= 100000U) &&
                       (get_ts_elapsed(now, hyundai_blinker_test_state.source[5].ts) >= 100000U) &&
                       (get_ts_elapsed(now, hyundai_blinker_test_state.source[6].ts) >= 100000U);
    const bool phase = ((hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_DISABLED) && quiet) ||
                       ((hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_PULSE) && (get_ts_elapsed(now, hyundai_blinker_test_state.pulse_ts) <= 2000000U));
    const bool direction_ok = (i == 0) || (direction == 3U) || (direction == 4U);
    tx = (off || rate) && counter && hyundai_blinker_test_same_body(p, 5 + i, i == 1) &&
         (off || (running && phase && direction_ok && ((hyundai_blinker_test_state.direction == 0U) || (hyundai_blinker_test_state.direction == direction) || (i == 0))));
    if (tx) {
      hyundai_blinker_test_state.spas_sent[i] = true;
      hyundai_blinker_test_state.spas_counter[i] = GET_BYTE(p, 2);
      hyundai_blinker_test_state.spas_tx_ts[i] = now;
      if (!off && (hyundai_blinker_test_state.phase == HYUNDAI_BLINKER_DISABLED)) {
        hyundai_blinker_test_state.phase = HYUNDAI_BLINKER_PULSE;
        hyundai_blinker_test_state.pulse_ts = now;
      }
      if ((i == 1) && !off) hyundai_blinker_test_state.direction = direction;
    }
  }

  if ((addr == 0x1EA) && (GET_LEN(p) == 32)) {
    const uint8_t hold = GET_BYTE(p, 3) & 0x60U;
    const bool off = hold == 0U;
    const bool direction = ((hold == 0x20U) && (hyundai_blinker_test_state.direction == 3U)) ||
                           ((hold == 0x40U) && (hyundai_blinker_test_state.direction == 4U));
    const bool phase = hyundai_blinker_test_state.phase >= HYUNDAI_BLINKER_PULSE;
    const bool rate = !hyundai_blinker_test_state.hold_sent || (get_ts_elapsed(now, hyundai_blinker_test_state.hold_tx_ts) >= 50000U);
    tx = hyundai_blinker_test_adrv_command(p, now, off) &&
         (off || (rate && running && phase && direction && hyundai_blinker_test_fresh(7, now, 100000U)));
    if (tx) {
      hyundai_blinker_test_state.hold_sent = !off;
      hyundai_blinker_test_state.hold_tx_ts = now;
      if (off) hyundai_blinker_test_state.direction = 0U;
      // Receive HOLD as a bounded command, never transmit the host's older
      // snapshot. The CAN driver consumes this flag instead of queueing it.
      extern bool safety_tx_buffered_for_fwd;
      safety_tx_buffered_for_fwd = true;
    }
  }
  return tx;
}

static int hyundai_blinker_test_fwd(CANPacket_t *p) {
  const int bus = GET_BUS(p);
  const uint32_t now = microsecond_timer_get();
  // Forward each CURRENT stock frame with its own counter and body. Forwarding
  // runs before rx_hook in the driver, so validate this frame independently.
  if ((bus == 2) && (GET_ADDR(p) == 0x1EA) && (GET_LEN(p) == 32) && hyundai_blinker_test_state.hold_sent &&
      (get_ts_elapsed(now, hyundai_blinker_test_state.hold_tx_ts) <= 150000U) && hyundai_blinker_test_running(now) &&
      hyundai_blinker_test_fresh(7, now, 100000U) && hyundai_blinker_test_crc(p)) {
    const uint8_t delta = GET_BYTE(p, 2) - hyundai_blinker_test_state.source[7].counter;
    if ((delta == 1U) || (delta == 2U)) {
      const uint8_t hold = (hyundai_blinker_test_state.direction == 3U) ? 0x20U : 0x40U;
      p->data[3] = (p->data[3] & 0x9FU) | hold;
      const uint16_t crc = (uint16_t)hyundai_common_canfd_compute_checksum(p);
      p->data[0] = (uint8_t)(crc & 255U);
      p->data[1] = (uint8_t)(crc >> 8U);
    }
  }
  return (bus == 0) ? 2 : ((bus == 2) ? 0 : -1);
}
