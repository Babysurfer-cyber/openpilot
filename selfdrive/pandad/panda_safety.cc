#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

void PandaSafety::configureSafetyMode() {
  bool is_onroad = params_.getBool("IsOnroad");

  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      cacheNormalSafetyMode(car_params);
      // A request left over from an earlier card/pandad instance is not a new
      // authorization to pause an ECU. A new session must request a new token.
      blinker_safety_transition_.reset(params_.get("HyundaiBlinkerTestSafetyRequest"));
      params_.remove("HyundaiBlinkerTestSafetyAck");
      safety_configured_ = true;
    }
  } else if (is_onroad && safety_configured_) {
    configureBlinkerTest();
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
    normal_car_params_.clear();
    blinker_test_supported_ = false;
    normal_hyundai_safety_param_ = 0U;
    blinker_safety_transition_.reset(params_.get("HyundaiBlinkerTestSafetyRequest"));
    if (!params_.get("HyundaiBlinkerTestSafetyAck").empty()) params_.remove("HyundaiBlinkerTestSafetyAck");
  }
}

void PandaSafety::cacheNormalSafetyMode(const std::string &params_string) {
  // Preserve the exact original configuration, including all alternative
  // experiences and every other Panda's safety entry. Never rewrite CarParams.
  normal_car_params_ = params_string;
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  auto cp = cmsg.getRoot<cereal::CarParams>();
  auto configs = cp.getSafetyConfigs();
  HyundaiBlinkerSafetyEligibility eligibility;
  eligibility.brand = cp.getBrand().cStr();
  eligibility.fingerprint = cp.getCarFingerprint().cStr();
  eligibility.flags = cp.getFlags();
  eligibility.passive = cp.getPassive();
  eligibility.dashcam_only = cp.getDashcamOnly();
  eligibility.panda_count = pandas_.size();
  eligibility.safety_config_count = configs.size();
  if (configs.size() == 1) {
    eligibility.hyundai_canfd_mode = configs[0].getSafetyModel() == cereal::CarParams::SafetyModel::HYUNDAI_CANFD;
    eligibility.safety_param = configs[0].getSafetyParam();
  }
  blinker_test_supported_ = eligibility.supported();
  normal_hyundai_safety_param_ = eligibility.safety_param;
}

void PandaSafety::configureBlinkerTest() {
  const auto request = params_.get("HyundaiBlinkerTestSafetyRequest");
  const auto action = blinker_safety_transition_.request(request, blinker_test_supported_);
  if (action == HyundaiBlinkerSafetyTransition::Action::TEST) {
    LOGW("Entering parked EV6 blinker safety session %s", blinker_safety_transition_.activeNonce().c_str());
    pandas_[0]->set_safety_model(cereal::CarParams::SafetyModel::HYUNDAI_CANFD, normal_hyundai_safety_param_ | 1024U);
  } else if (action == HyundaiBlinkerSafetyTransition::Action::NORMAL) {
    LOGW("Restoring original Panda safety after parked EV6 blinker test");
    setSafetyMode(normal_car_params_);
  } else if (action == HyundaiBlinkerSafetyTransition::Action::ERROR) {
    LOGW("Rejected parked EV6 blinker safety request");
  }
  // This acknowledges that the USB command was issued, not that hardware
  // applied it. The Python coordinator separately checks new pandaStates.
  const auto &ack = blinker_safety_transition_.ack();
  if (!ack.empty() && params_.get("HyundaiBlinkerTestSafetyAck") != ack) {
    params_.put("HyundaiBlinkerTestSafetyAck", ack);
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    for (int i = 0; i < pandas_.size(); ++i) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    }
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    for (int i = 0; i < pandas_.size(); ++i) {
      const uint16_t safety_param = (i > 0 || !obd_multiplexing_requested) ? 1U : 0U;
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    }
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

std::string PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return params_.get("CarParams");
}

void PandaSafety::setSafetyMode(const std::string &params_string) {
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();

  for (int i = 0; i < pandas_.size(); ++i) {
    // Default to SILENT safety model if not specified
    cereal::CarParams::SafetyModel safety_model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t safety_param = 0U;
    if (i < safety_configs.size()) {
      safety_model = safety_configs[i].getSafetyModel();
      safety_param = safety_configs[i].getSafetyParam();
    }

    LOGW("Panda %d: setting safety model: %d, param: %d, alternative experience: %d", i, (int)safety_model, safety_param, alternative_experience);
    pandas_[i]->set_alternative_experience(alternative_experience);
    pandas_[i]->set_safety_model(safety_model, safety_param);
  }
}
