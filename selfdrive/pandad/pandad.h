#pragma once

#include <string>
#include <vector>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"
#include "selfdrive/pandad/hyundai_blinker_safety.h"

void pandad_main_thread(std::vector<std::string> serials);

class PandaSafety {
public:
  PandaSafety(const std::vector<Panda *> &pandas) : pandas_(pandas) {}
  void configureSafetyMode();

private:
  void updateMultiplexingMode();
  std::string fetchCarParams();
  void setSafetyMode(const std::string &params_string);
  void cacheNormalSafetyMode(const std::string &params_string);
  void configureBlinkerTest();

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  std::string normal_car_params_;
  bool blinker_test_supported_ = false;
  uint16_t normal_hyundai_safety_param_ = 0U;
  HyundaiBlinkerSafetyTransition blinker_safety_transition_;
  std::vector<Panda *> pandas_;
  Params params_;
};
