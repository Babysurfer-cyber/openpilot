#!/usr/bin/env python3
"""Compile the *actual* EV6 parked-input guard extracted from panda source.

Stand-alone ABI/logic smoke check: it does NOT compile the full panda safety
library and is not a substitute for that library's integration tests.
"""
import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

DEFAULT_HEADER = Path(__file__).resolve().parents[1] / "safety" / "safety_hyundai_canfd.h"

PREFIX = r"""
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef struct { int addr; int bus; } CanMsgCheck;
typedef struct {
  bool msg_seen;
  int index;
  bool valid_checksum;
  int wrong_counters;
  bool valid_quality_flag;
  uint32_t last_timestamp;
  bool lagging;
} RxStatus;
typedef struct { CanMsgCheck msg[3]; RxStatus status; } RxCheck;
typedef struct { RxCheck *rx_checks; int rx_checks_len; } safety_config;
extern safety_config current_safety_config;
extern const int MAX_WRONG_COUNTERS;
static uint32_t get_ts_elapsed(uint32_t now, uint32_t then) { return now - then; }
"""

SUFFIX = r"""
static RxCheck stock[3];
safety_config current_safety_config = {stock, 3};
const int MAX_WRONG_COUNTERS = 5;
static int assertions = 0;
#define CHECK(want) do { if (!(want)) { fprintf(stderr, "failed case #%d line %d\n", assertions, __LINE__); return 1; } assertions++; } while (0)
static void valid_setup(void) {
  memset(stock, 0, sizeof(stock));
  const int ids[] = {0x35, 0x175, 0xA0};
  for (int k = 0; k < 3; ++k) {
    stock[k].msg[0].addr = ids[k];
    stock[k].msg[0].bus = 0;
    stock[k].status.msg_seen = true;
    stock[k].status.index = 0;
    stock[k].status.valid_checksum = true;
    stock[k].status.valid_quality_flag = true;
    stock[k].status.wrong_counters = 0;
    stock[k].status.lagging = false;
    stock[k].status.last_timestamp = 199000;
  }
  current_safety_config.rx_checks_len = 3;
}
int main(void) {
  const uint32_t now = 200000;
  valid_setup(); CHECK(hyundai_ev6_parked_inputs_valid(now));
  for (int i = 0; i < 3; ++i) {
    valid_setup(); stock[i].status.msg_seen = false; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].msg[0].bus = 1; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].status.valid_checksum = false; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].status.valid_quality_flag = false; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].status.wrong_counters = 5; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].status.lagging = true; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].status.last_timestamp = 70000; CHECK(!hyundai_ev6_parked_inputs_valid(now));
    valid_setup(); stock[i].msg[0].addr += 1; CHECK(!hyundai_ev6_parked_inputs_valid(now));
  }
  valid_setup(); current_safety_config.rx_checks_len = 0; CHECK(!hyundai_ev6_parked_inputs_valid(now));
  valid_setup(); stock[2].status.last_timestamp = 80000; CHECK(hyundai_ev6_parked_inputs_valid(now));
  valid_setup(); stock[2].status.last_timestamp = 79999; CHECK(!hyundai_ev6_parked_inputs_valid(now));
  valid_setup(); stock[0].status.wrong_counters = 4; CHECK(hyundai_ev6_parked_inputs_valid(now));
  valid_setup(); stock[1].msg[1].addr = 0x175; stock[1].msg[1].bus = 0; stock[1].status.index = 1;
  CHECK(hyundai_ev6_parked_inputs_valid(now));
  printf("PASS: %d compiled C source-contract checks; no vehicle communication\n", assertions);
  return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--header", type=Path, default=DEFAULT_HEADER)
    args = parser.parse_args()
    source = args.header.read_text(encoding="utf-8")
    marker = "static bool hyundai_ev6_parked_inputs_valid(uint32_t now) {"
    pos = source.find(marker)
    if pos < 0 or source.find(marker, pos + 1) >= 0:
        raise RuntimeError("exactly one production EV6 parked-input guard required")
    ending = source.find("\n}\n", pos)
    if ending < 0:
        raise RuntimeError("incomplete production guard function")
    guard = source[pos:ending + 3]
    if not all(token in guard for token in ("0x35", "0x175", "0xA0", "valid_checksum", "wrong_counters", "120000U")):
        raise RuntimeError("guard contract changed: review compiler test")
    compiler = shutil.which("gcc")
    if compiler is None:
        raise RuntimeError("gcc is required; cannot claim a test pass")
    with tempfile.TemporaryDirectory(prefix="ev6_park_gate_") as work:
        src, exe = Path(work) / "gate.c", Path(work) / "gate"
        src.write_text(PREFIX + guard + SUFFIX, encoding="utf-8")
        subprocess.run([compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O1", str(src), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
