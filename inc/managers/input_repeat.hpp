#pragma once

#include <pad/pad.hpp>

/**
 * Menu-style input: same as Pad::getClicked(), but D-pad, L1 and R1 also
 * "click" again periodically while held (like a keyboard key repeat).
 * Call InputRepeat_update() once per frame before game states update.
 */
void InputRepeat_update(const Tyra::Pad& pad, const float& deltaTime);
const Tyra::PadButtons& InputRepeat_getClicked();
