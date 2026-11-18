#pragma once
#include <tamtypes.h>
#include "tyra"

using Tyra::Color;
using Tyra::Vec2;

enum class TextAlignment { Left, Center, Right };

class FontOptions {
 public:
  FontOptions(){};

  FontOptions(const Vec2& pos, const Color& color = Color(255, 255, 255),
              const float& scale = 1.0F,
              const TextAlignment& alignment = TextAlignment::Left,
              const float& maxWidth = 0.0F) {
    this->position.set(pos);
    this->color.set(color);
    this->scale = scale;
    this->alignment = alignment;
    this->maxWidth = maxWidth;
  };

  ~FontOptions(){};

  Vec2 position = Vec2(0.0f, 0.0f);
  Color color = Color(255, 255, 255);
  float scale = 1.0f;
  TextAlignment alignment = TextAlignment::Left;
  /**
   * If > 0, the text is shrunk (never enlarged) so it is at most this wide.
   * Long translations then still fit in their button/box.
   */
  float maxWidth = 0.0F;
};
