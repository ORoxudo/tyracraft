#pragma once

#include "tyra"
#include <tamtypes.h>
#include "managers/clipper/custom_planes_clip_algorithm.hpp"
#include "utils.hpp"

using Tyra::Color;
using Tyra::EEClipAlgorithmSettings;
using Tyra::Plane;
using Tyra::PlanesClipAlgorithm;
using Tyra::PlanesClipVertex;
using Tyra::PlanesClipVertexPtrs;
using Tyra::RendererSettings;
using Tyra::Vec4;

/**
 * @brief Clips `count` vertices (whole triangles) against the frustum and
 * APPENDS the result to the out vectors. Works directly on the source arrays
 * (no copies) and reuses an internal scratch buffer, so it doesn't allocate
 * per call once the vectors have grown.
 *
 * @return number of vertices appended
 */
inline int ClippingManager_ClipMesh(const Vec4* vert, const Vec4* sts,
                                    const Color* in_colors, const u32 count,
                                    std::vector<Vec4>& out_vertex,
                                    std::vector<Vec4>& out_uv,
                                    std::vector<Color>& out_colors,
                                    Plane* frustumPlanes, const Vec4& camPos) {
  static std::vector<PlanesClipVertex> clippedTriangle;
  static CustomPlanesClipAlgorithm algorithm;

  EEClipAlgorithmSettings algoSettings;
  algoSettings = {false, sts != nullptr, in_colors != nullptr};

  Vec4 camPosCopy = camPos;
  Vec4 inputVerts[3];
  std::array<PlanesClipVertexPtrs, 3> inputTriangle;

  // The clip algorithm takes non-const pointers but doesn't write to them
  Vec4* uv = const_cast<Vec4*>(sts);
  Vec4* colors = reinterpret_cast<Vec4*>(const_cast<Color*>(in_colors));

  int result = 0;

  for (u32 i = 0; i < count / 3; i++) {
    for (u8 j = 0; j < 3; j++) {
      inputVerts[j] = vert[i * 3 + j];
      inputTriangle[j] = {&inputVerts[j], nullptr, &uv[i * 3 + j],
                          &colors[i * 3 + j]};
    }

    if (
        // Back face culling
        !Vec4::shouldBeBackfaceCulled(&camPosCopy, &inputVerts[0],
                                      &inputVerts[1], &inputVerts[2]) ||
        // Only triangles crossing the frustum need clipping
        Utils::FrustumTriangleIntersect(frustumPlanes, inputVerts[0],
                                        inputVerts[1], inputVerts[2]) !=
            Tyra::CoreBBoxFrustum::PARTIALLY_IN_FRUSTUM) {
      continue;
    }

    const int clippedCount = algorithm.clip(
        clippedTriangle, inputTriangle.data(), algoSettings, frustumPlanes);
    if (clippedCount == 0) continue;

    result += clippedCount;

    for (size_t k = 0; k < clippedTriangle.size(); k++) {
      out_vertex.emplace_back(clippedTriangle[k].position);
      out_uv.emplace_back(clippedTriangle[k].st);
      out_colors.emplace_back(clippedTriangle[k].color.xyzw);
    }
  }

  return result;
}
