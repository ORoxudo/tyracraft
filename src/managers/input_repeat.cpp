#include "managers/input_repeat.hpp"

using Tyra::Pad;
using Tyra::PadButtons;

namespace {

constexpr float REPEAT_FIRST_DELAY = 0.35F;  // seconds before repeating
constexpr float REPEAT_INTERVAL = 0.09F;     // seconds between repeats

PadButtons g_navClicked = {};

struct RepeatTimer {
  float remaining = 0.0F;

  u8 update(const u8 clicked, const u8 pressed, const float& dt) {
    if (clicked) {
      remaining = REPEAT_FIRST_DELAY;
      return 1;
    }
    if (!pressed) {
      remaining = 0.0F;
      return 0;
    }
    remaining -= dt;
    if (remaining <= 0.0F) {
      remaining += REPEAT_INTERVAL;
      if (remaining <= 0.0F) remaining = REPEAT_INTERVAL;
      return 1;
    }
    return 0;
  }
};

RepeatTimer up, down, left, right, l1, r1;

}  // namespace

void InputRepeat_update(const Pad& pad, const float& deltaTime) {
  const PadButtons& clicked = pad.getClicked();
  const PadButtons& pressed = pad.getPressed();

  g_navClicked = clicked;
  g_navClicked.DpadUp =
      up.update(clicked.DpadUp, pressed.DpadUp, deltaTime);
  g_navClicked.DpadDown =
      down.update(clicked.DpadDown, pressed.DpadDown, deltaTime);
  g_navClicked.DpadLeft =
      left.update(clicked.DpadLeft, pressed.DpadLeft, deltaTime);
  g_navClicked.DpadRight =
      right.update(clicked.DpadRight, pressed.DpadRight, deltaTime);
  g_navClicked.L1 = l1.update(clicked.L1, pressed.L1, deltaTime);
  g_navClicked.R1 = r1.update(clicked.R1, pressed.R1, deltaTime);
}

const PadButtons& InputRepeat_getClicked() { return g_navClicked; }
