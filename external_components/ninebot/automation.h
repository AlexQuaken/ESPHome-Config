#pragma once

#include "esphome/core/automation.h"
#include "ninebot.h"

#ifdef USE_ESP32

namespace esphome::ninebot {

template<typename... Ts> class PairAction : public Action<Ts...> {
 public:
  explicit PairAction(NinebotHub *hub) : hub_(hub) {}
  void play(const Ts &...x) override { this->hub_->start_pairing(); }

 protected:
  NinebotHub *hub_;
};

template<typename... Ts> class ForgetAction : public Action<Ts...> {
 public:
  explicit ForgetAction(NinebotHub *hub) : hub_(hub) {}
  void play(const Ts &...x) override { this->hub_->forget_pairing(); }

 protected:
  NinebotHub *hub_;
};

template<typename... Ts> class PollAction : public Action<Ts...> {
 public:
  explicit PollAction(NinebotHub *hub) : hub_(hub) {}
  void play(const Ts &...x) override { this->hub_->poll_now(); }

 protected:
  NinebotHub *hub_;
};

template<typename... Ts> class PauseAction : public Action<Ts...> {
 public:
  explicit PauseAction(NinebotHub *hub) : hub_(hub) {}
  TEMPLATABLE_VALUE(bool, paused)
  void play(const Ts &...x) override { this->hub_->set_paused(this->paused_.value(x...)); }

 protected:
  NinebotHub *hub_;
};

}  // namespace esphome::ninebot

#endif
