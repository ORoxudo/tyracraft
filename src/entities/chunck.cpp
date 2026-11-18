#include "entities/chunck.hpp"
#include <vector>
#include <functional>
#include <iterator>
#include <algorithm>
#include "managers/light_manager.hpp"
#include "managers/mesh/mesh_builder.hpp"
#include "managers/collision_manager.hpp"
#include "managers/clipping_manager.hpp"
#include <dma.h>
#include <cmath>

float Chunck::fogStart = 0.0F;
float Chunck::fogEnd = 0.0F;

/**
 * The pipeline sends its last DMA packet asynchronously and that packet
 * points straight at our arrays. Wait for it before reusing a scratch buffer,
 * otherwise the previous draw could read the next chunk's data.
 */
static inline void Chunck_waitLastDraw() {
  dma_channel_wait(DMA_CHANNEL_VIF1, 0);
}

/** True when the farthest corner of the chunk can be inside the fog */
static bool Chunck_canBeInFog(const Vec4& cam, const Vec4& min,
                              const Vec4& max) {
  if (Chunck::fogEnd <= 0.0F) return false;
  const float fx =
      std::max(std::fabs(cam.x - min.x), std::fabs(cam.x - max.x)) +
      DUBLE_BLOCK_SIZE;
  const float fy =
      std::max(std::fabs(cam.y - min.y), std::fabs(cam.y - max.y)) +
      DUBLE_BLOCK_SIZE;
  const float fz =
      std::max(std::fabs(cam.z - min.z), std::fabs(cam.z - max.z)) +
      DUBLE_BLOCK_SIZE;
  return fx * fx + fy * fy + fz * fz > Chunck::fogStart * Chunck::fogStart;
}

/**
 * Copies `colors` into `out`, fading the alpha of the vertices that are
 * inside the fog so they blend into the sky behind them.
 * Returns false if no vertex was inside the fog (then `out` isn't needed).
 */
static bool Chunck_applyFog(const std::vector<Vec4>& verts,
                            const std::vector<Color>& colors, const Vec4& cam,
                            std::vector<Color>& out) {
  const size_t count = std::min(verts.size(), colors.size());
  const float start = Chunck::fogStart;
  const float startSq = start * start;
  const float invRange = 1.0F / (Chunck::fogEnd - start);
  bool hasFog = false;

  out.resize(count);
  for (size_t i = 0; i < count; i++) {
    const float dx = verts[i].x - cam.x;
    const float dy = verts[i].y - cam.y;
    const float dz = verts[i].z - cam.z;
    const float distSq = dx * dx + dy * dy + dz * dz;

    out[i] = colors[i];
    if (distSq > startSq) {
      float t = (std::sqrt(distSq) - start) * invRange;
      if (t > 1.0F) t = 1.0F;
      out[i].a = colors[i].a * (1.0F - t);
      hasFog = true;
    }
  }

  return hasFog;
}

Chunck::Chunck(const Vec4& minOffset, const Vec4& maxOffset, const u16& id) {
  this->id = id;
  this->minOffset.set(minOffset);
  this->maxOffset.set(maxOffset);
  this->center.set((maxOffset + minOffset) / 2);
  tempLoadingOffset.set(minOffset);

  // Blocks are centered on offset * DUBLE_BLOCK_SIZE (they span +-BLOCK_SIZE),
  // so the chunk geometry goes from min*16 - 8 to max*16 - 8. Without this
  // shift the frustum check was half a block off and chunks at the screen
  // edges could be skipped (holes in the ground).
  const Vec4 halfBlock = Vec4(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
  const Vec4 tempMin = minOffset * DUBLE_BLOCK_SIZE - halfBlock;
  const Vec4 tempMax = maxOffset * DUBLE_BLOCK_SIZE - halfBlock;
  scaledMinOffset.set(tempMin);
  scaledMaxOffset.set(tempMax);

  u32 count = 8;
  Vec4 _vertices[count] = {
      Vec4(tempMin),
      Vec4(tempMax.x, tempMin.y, tempMin.z),
      Vec4(tempMin.x, tempMax.y, tempMin.z),
      Vec4(tempMin.x, tempMin.y, tempMax.z),
      Vec4(tempMax),
      Vec4(tempMin.x, tempMax.y, tempMax.z),
      Vec4(tempMax.x, tempMin.y, tempMax.z),
      Vec4(tempMax.x, tempMax.y, tempMin.z),
  };
  this->bbox = new BBox(_vertices, count);
};

Chunck::~Chunck() {
  clear();
  delete bbox;
};

void Chunck::init(LevelMap* t_terrain, WorldLightModel* t_worldLightModel) {
  this->t_terrain = t_terrain;
  this->t_worldLightModel = t_worldLightModel;
}

void Chunck::update(const Plane* frustumPlanes) {
  updateFrustumCheck(frustumPlanes);

  if (isDrawDataLoaded() && frustumCheck == Tyra::PARTIALLY_IN_FRUSTUM &&
      _distanceFromPlayerInChunks > -1 && _distanceFromPlayerInChunks < 3) {
    updateSurroundingBlocks();
  }
}

void Chunck::updateSurroundingBlocks() {
  surroundingBlocks.clear();
  surroundingTransparentBlocks.clear();

  Vec4 tempOffset = camPositon / DUBLE_BLOCK_SIZE;
  Vec4 camOffset = Vec4(std::lrint(tempOffset.x), std::lrint(tempOffset.y),
                        std::lrint(tempOffset.z));

  // Blocks near the camera whose triangles can cross the camera plane get
  // clipped on the CPU (the VU1 program just drops those triangles).
  // The camera is ~2 blocks above the floor block, so look 3 blocks down:
  // with only 1 the ground right in front of the feet had missing pieces.
  for (s8 _x = -3; _x <= 3; _x++) {
    for (s8 _y = -3; _y <= 2; _y++) {
      for (s8 _z = -3; _z <= 3; _z++) {
        Vec4 _bOffset = camOffset + Vec4(_x, _y, _z);

        // Skip offsets outside this chunk: getBlockById() is a linear search
        // over all chunk blocks and would scan everything for nothing.
        if (!containsBlock(&_bOffset)) continue;

        u32 blockIndex = getIndexByOffset(_bOffset.x, _bOffset.y, _bOffset.z);
        Block* t_block = getBlockById(blockIndex);

        if (t_block &&
            Utils::FrustumAABBIntersect(frustumPlanes, &t_block->minCorner,
                                        &t_block->maxCorner) ==
                CoreBBoxFrustum::PARTIALLY_IN_FRUSTUM) {
          if (t_block->hasTransparency) {
            surroundingTransparentBlocks.push_back(t_block);
          } else {
            surroundingBlocks.push_back(t_block);
          }
        }
      }
    }
  }
}

void Chunck::renderer(Renderer* t_renderer, StaticPipeline* stapip,
                      BlockManager* t_blockManager) {
  if (isDrawDataLoaded()) {
    StaPipTextureBag textureBag;
    StaPipInfoBag infoBag;
    StaPipColorBag colorBag;
    StaPipBag bag;

    textureBag.coordinates = uvMap.data();
    textureBag.texture = t_blockManager->getBlocksTexture();

    infoBag.textureMappingType = Tyra::PipelineTextureMappingType::TyraNearest;
    infoBag.shadingType = Tyra::PipelineShadingType::TyraShadingGouraud;
    infoBag.blendingEnabled = false;
    infoBag.antiAliasingEnabled = false;
    infoBag.fullClipChecks = false;
    infoBag.frustumCulling =
        Tyra::PipelineInfoBagFrustumCulling::PipelineInfoBagFrustumCulling_None;

    colorBag.many = verticesColors.data();

    // Distance fog: fade far blocks into the sky
    static std::vector<Color> foggedColors;
    if (Chunck_canBeInFog(camPositon, scaledMinOffset, scaledMaxOffset)) {
      Chunck_waitLastDraw();
      if (Chunck_applyFog(vertices, verticesColors, camPositon,
                          foggedColors)) {
        colorBag.many = foggedColors.data();
        infoBag.blendingEnabled = true;
      }
    }

    bag.count = vertices.size();
    bag.vertices = vertices.data();
    bag.color = &colorBag;
    bag.info = &infoBag;
    bag.texture = &textureBag;

    t_renderer->renderer3D.usePipeline(stapip);

    M4x4 rawMatrix = M4x4::Identity;
    infoBag.model = &rawMatrix;

    stapip->core.render(&bag);
    // t_renderer->renderer3D.utility.drawBBox(*bbox, Color(255, 0, 0));

    if (surroundingBlocks.size() > 0) {
      renderSolidPartialBlocks(t_renderer, stapip, t_blockManager);
    }
  }
};

void Chunck::rendererTransparentData(Renderer* t_renderer,
                                     StaticPipeline* stapip,
                                     BlockManager* t_blockManager) {
  if (isDrawDataLoaded()) {
    StaPipTextureBag textureBag;
    StaPipInfoBag infoBag;
    StaPipColorBag colorBag;
    StaPipBag bag;

    textureBag.coordinates = uvMapWithTransparency.data();
    textureBag.texture = t_blockManager->getBlocksTexture();

    infoBag.textureMappingType = Tyra::PipelineTextureMappingType::TyraNearest;
    infoBag.shadingType = Tyra::PipelineShadingType::TyraShadingGouraud;
    infoBag.blendingEnabled = true;
    infoBag.antiAliasingEnabled = false;
    infoBag.fullClipChecks = false;
    infoBag.frustumCulling =
        Tyra::PipelineInfoBagFrustumCulling::PipelineInfoBagFrustumCulling_None;

    colorBag.many = verticesColorsWithTransparency.data();

    // Distance fog (see renderer())
    static std::vector<Color> foggedColors;
    if (Chunck_canBeInFog(camPositon, scaledMinOffset, scaledMaxOffset)) {
      Chunck_waitLastDraw();
      if (Chunck_applyFog(verticesWithTransparency,
                          verticesColorsWithTransparency, camPositon,
                          foggedColors)) {
        colorBag.many = foggedColors.data();
      }
    }

    bag.count = verticesWithTransparency.size();
    bag.vertices = verticesWithTransparency.data();
    bag.color = &colorBag;
    bag.info = &infoBag;
    bag.texture = &textureBag;

    t_renderer->renderer3D.usePipeline(stapip);

    M4x4 rawMatrix = M4x4::Identity;
    infoBag.model = &rawMatrix;

    stapip->core.render(&bag);
    // t_renderer->renderer3D.utility.drawBBox(*bbox, Color(255, 0, 0));

    if (surroundingTransparentBlocks.size() > 0) {
      renderTransparentPartialBlocks(t_renderer, stapip, t_blockManager);
    }
  }
};

void Chunck::renderSolidPartialBlocks(Renderer* t_renderer,
                                      StaticPipeline* stapip,
                                      BlockManager* t_blockManager) {
  renderPartialBlocks(t_renderer, false, stapip, t_blockManager,
                      surroundingBlocks, vertices, uvMap, verticesColors);
}

void Chunck::renderTransparentPartialBlocks(Renderer* t_renderer,
                                            StaticPipeline* stapip,
                                            BlockManager* t_blockManager) {
  renderPartialBlocks(t_renderer, true, stapip, t_blockManager,
                      surroundingTransparentBlocks, verticesWithTransparency,
                      uvMapWithTransparency, verticesColorsWithTransparency);
}

void Chunck::renderPartialBlocks(Renderer* t_renderer, u8 hasTransparency,
                                 StaticPipeline* stapip,
                                 BlockManager* t_blockManager,
                                 const std::vector<Block*>& t_blocks,
                                 const std::vector<Vec4>& srcVertices,
                                 const std::vector<Vec4>& srcUVs,
                                 const std::vector<Color>& srcColors) {
  // Shared scratch buffers: rendering is single threaded and the pipeline
  // consumes the data inside render(), so they can be reused every call
  // without allocating again once they've grown.
  static std::vector<Vec4> outVertices;
  static std::vector<Vec4> outUVMap;
  static std::vector<Color> outColors;

  // The previous partial draw may still be reading these buffers
  Chunck_waitLastDraw();

  outVertices.clear();
  outUVMap.clear();
  outColors.clear();

  Plane* frustumPlanes =
      (Plane*)t_renderer->core.renderer3D.frustumPlanes.getAll();
  const size_t total = srcVertices.size();

  for (size_t i = 0; i < t_blocks.size(); i++) {
    const Block* t_block = t_blocks[i];
    const size_t start = t_block->drawDataIndex;
    size_t count = t_block->drawDataLength;

    if (start >= total) continue;
    if (start + count > total) count = total - start;

    ClippingManager_ClipMesh(&srcVertices[start], &srcUVs[start],
                             &srcColors[start], count, outVertices, outUVMap,
                             outColors, frustumPlanes, camPositon);
  }

  if (outVertices.empty()) return;

  StaPipTextureBag textureBag;
  StaPipInfoBag infoBag;
  StaPipColorBag colorBag;
  StaPipBag bag;

  textureBag.coordinates = outUVMap.data();
  textureBag.texture = t_blockManager->getBlocksTexture();

  infoBag.textureMappingType = Tyra::PipelineTextureMappingType::TyraNearest;
  infoBag.shadingType = Tyra::PipelineShadingType::TyraShadingGouraud;
  infoBag.blendingEnabled = hasTransparency;
  infoBag.antiAliasingEnabled = false;

  infoBag.fullClipChecks = false;
  infoBag.frustumCulling =
      Tyra::PipelineInfoBagFrustumCulling::PipelineInfoBagFrustumCulling_None;

  colorBag.many = outColors.data();

  bag.count = outVertices.size();
  bag.vertices = outVertices.data();
  bag.color = &colorBag;
  bag.info = &infoBag;
  bag.texture = &textureBag;

  t_renderer->renderer3D.usePipeline(stapip);

  M4x4 rawMatrix = M4x4::Identity;
  infoBag.model = &rawMatrix;

  stapip->core.render(&bag);
}

void Chunck::clear() {
  clearDrawData();

  visibleFacesCount = 0;
  blocksCount = 0;

  for (u16 i = 0; i < blocks.size(); i++) {
    // May already be freed by an interrupted clearAsync()
    if (!blocks[i]) continue;

    g_AABBTree->remove(blocks[i]->tree_index);

    delete blocks[i];
    blocks[i] = nullptr;
  }

  blocks.clear();
  blocks.shrink_to_fit();
  _isPreAllocated = false;
  _unloaderBatchCounter = 0;

  surroundingBlocks.clear();
  surroundingBlocks.shrink_to_fit();

  surroundingTransparentBlocks.clear();
  surroundingTransparentBlocks.shrink_to_fit();

  resetLoadingOffset();

  this->state = ChunkState::Clean;
}

void Chunck::clearAsync() {
  if (state != ChunkState::Unloading) {
    // From now on blocks get freed in batches: stop rendering, updating and
    // reading this chunk's blocks so nothing touches freed memory.
    state = ChunkState::Unloading;
    _isDrawDataLoaded = false;
    surroundingBlocks.clear();
    surroundingTransparentBlocks.clear();
    _unloaderBatchCounter = 0;
  }

  size_t counter = 0;
  for (size_t i = _unloaderBatchCounter; i < blocks.size(); i++) {
    if (blocks[i]) {
      g_AABBTree->remove(blocks[i]->tree_index);
      delete blocks[i];
      blocks[i] = nullptr;
    }

    if (counter >= UNLOAD_CHUNK_BATCH) {
      _unloaderBatchCounter = i + 1;
      return;
    } else {
      counter++;
    }
  }

  clearDrawData();

  visibleFacesCount = 0;
  blocksCount = 0;

  blocks.clear();
  blocks.shrink_to_fit();
  _isPreAllocated = false;

  resetLoadingOffset();
  _unloaderBatchCounter = 0;

  state = ChunkState::Clean;
}

// void Chunck::updateBlocks(const Vec4& playerPosition) {}

void Chunck::clearDrawData() {
  vertices.clear();
  vertices.shrink_to_fit();
  verticesColors.clear();
  verticesColors.shrink_to_fit();
  uvMap.clear();
  uvMap.shrink_to_fit();

  verticesWithTransparency.clear();
  verticesWithTransparency.shrink_to_fit();
  verticesColorsWithTransparency.clear();
  verticesColorsWithTransparency.shrink_to_fit();
  uvMapWithTransparency.clear();
  uvMapWithTransparency.shrink_to_fit();

  surroundingBlocks.clear();
  surroundingTransparentBlocks.clear();

  _isDrawDataLoaded = false;
  _isMemoryReserved = false;
  _isPreAllocated = false;

  _loaderBatchCounter = 0;
  _unloaderBatchCounter = 0;
}

void Chunck::clearDrawDataWithoutShrink() {
  vertices.clear();
  verticesColors.clear();
  uvMap.clear();

  verticesWithTransparency.clear();
  verticesColorsWithTransparency.clear();
  uvMapWithTransparency.clear();

  _isDrawDataLoaded = false;
}

void Chunck::loadDrawDataWithoutSorting() {
  vertices.reserve(visibleFacesCount);
  verticesColors.reserve(visibleFacesCount);
  uvMap.reserve(visibleFacesCount);

  verticesWithTransparency.reserve(visibleFacesCountWithTransparency);
  verticesColorsWithTransparency.reserve(visibleFacesCountWithTransparency);
  uvMapWithTransparency.reserve(visibleFacesCountWithTransparency);

  for (size_t i = 0; i < blocks.size(); i++) {
    if (blocks[i]->hasTransparency) {
      blocks[i]->drawDataIndex = verticesWithTransparency.size();

      MeshBuilder_BuildMesh(
          blocks[i], &verticesWithTransparency, &verticesColorsWithTransparency,
          &uvMapWithTransparency, t_worldLightModel, t_terrain);

      blocks[i]->drawDataLength =
          verticesWithTransparency.size() - blocks[i]->drawDataIndex;

      // printf("Transparent Block %i\n", (int)i);
      // printf("drawDataIndex %i | drawDataLength %i \n\n",
      //        blocks[i]->drawDataIndex, blocks[i]->drawDataLength);

    } else {
      blocks[i]->drawDataIndex = vertices.size();

      MeshBuilder_BuildMesh(blocks[i], &vertices, &verticesColors, &uvMap,
                            t_worldLightModel, t_terrain);

      blocks[i]->drawDataLength =
          vertices.size() - blocks[i]->drawDataIndex;

      // printf("Block %i\n", (int)i);
      // printf("drawDataIndex %i | drawDataLength %i \n\n",
      //        blocks[i]->drawDataIndex, blocks[i]->drawDataLength);
    }
  }

  _isDrawDataLoaded = true;
}

void Chunck::loadDrawDataAsync() {
  if (_isMemoryReserved == false) {
    vertices.reserve(visibleFacesCount);
    verticesColors.reserve(visibleFacesCount);
    uvMap.reserve(visibleFacesCount);

    verticesWithTransparency.reserve(visibleFacesCountWithTransparency);
    verticesColorsWithTransparency.reserve(visibleFacesCountWithTransparency);
    uvMapWithTransparency.reserve(visibleFacesCountWithTransparency);

    _isMemoryReserved = true;
  }

  size_t counter = 0;
  for (size_t i = _loaderBatchCounter; i < blocks.size(); i++) {
    if (blocks[i]->hasTransparency) {
      blocks[i]->drawDataIndex = verticesWithTransparency.size();

      MeshBuilder_BuildMesh(
          blocks[i], &verticesWithTransparency, &verticesColorsWithTransparency,
          &uvMapWithTransparency, t_worldLightModel, t_terrain);

      blocks[i]->drawDataLength =
          verticesWithTransparency.size() - blocks[i]->drawDataIndex;
    } else {
      blocks[i]->drawDataIndex = vertices.size();

      MeshBuilder_BuildMesh(blocks[i], &vertices, &verticesColors, &uvMap,
                            t_worldLightModel, t_terrain);

      blocks[i]->drawDataLength =
          vertices.size() - blocks[i]->drawDataIndex;
    }

    if (counter >= LOAD_CHUNK_BATCH) {
      _loaderBatchCounter = i + 1;
      return;
    } else {
      counter++;
    }
  }

  _isDrawDataLoaded = true;
  _loaderBatchCounter = 0;
}

void Chunck::loadDrawData() { loadDrawDataWithoutSorting(); }

void Chunck::reloadLightData() {
  // Only rebuild colors for chunks that currently have draw data: otherwise
  // the color arrays would be filled without vertices and be misaligned
  // once the mesh is built, and unloading chunks have freed blocks.
  if (state != ChunkState::Loaded || !isDrawDataLoaded()) return;

  verticesColors.clear();
  verticesColorsWithTransparency.clear();

  for (size_t i = 0; i < blocks.size(); i++) {
    if (blocks[i]->hasTransparency) {
      MeshBuilder_BuildLightData(blocks[i], &verticesColorsWithTransparency,
                                 t_worldLightModel, t_terrain);
    } else {
      MeshBuilder_BuildLightData(blocks[i], &verticesColors, t_worldLightModel,
                                 t_terrain);
    }
  }
}

// TODO: move to a block builder

void Chunck::updateFrustumCheck(const Plane* frustumPlanes) {
  this->frustumPlanes = (Plane*)frustumPlanes;
  this->frustumCheck = Utils::FrustumAABBIntersect(
      frustumPlanes, &scaledMinOffset, &scaledMaxOffset);
}

Block* Chunck::getBlockByPosition(const Vec4* pos) {
  for (size_t i = 0; i < blocks.size(); i++) {
    if (!blocks[i]) continue;
    const auto bPos = blocks[i]->position;
    if (bPos.x == pos->x && bPos.y == pos->y && bPos.z == pos->z)
      return blocks[i];
  }
  return nullptr;
}

Block* Chunck::getBlockByOffset(const Vec4* offset) {
  for (size_t i = 0; i < blocks.size(); i++) {
    if (!blocks[i]) continue;
    Vec4 _tempBlockOffset;
    GetXYZFromPos(&blocks[i]->offset, &_tempBlockOffset);

    if (_tempBlockOffset.x == offset->x && _tempBlockOffset.y == offset->y &&
        _tempBlockOffset.z == offset->z)
      return blocks[i];
  }
  return nullptr;
}

Block* Chunck::getBlockById(const u32 blockId) {
  for (size_t i = 0; i < blocks.size(); i++) {
    if (blocks[i] && blocks[i]->index == blockId) return blocks[i];
  }
  return nullptr;
}

void Chunck::removeBlock(Block* target) {
  blocks.erase(
      std::remove_if(blocks.begin(), blocks.end(),
                     [target](Block* b) { return b && b->index == target->index; }),
      blocks.end());
  g_AABBTree->remove(target->tree_index);
  delete target;
}

void Chunck::removeBlockByOffset(u32 offset) {
  Vec4 _offsetVec;
  GetXYZFromPos(&offset, &_offsetVec);
  Block* target = getBlockByOffset(&_offsetVec);

  if (target) removeBlock(target);
}

void Chunck::removeBlockByOffset(Vec4* offset) {
  Block* target = getBlockByOffset(offset);
  if (target) removeBlock(target);
}

void Chunck::removeBlockByLocalIndex(u16 index) {
  TYRA_ASSERT(index < blocks.size(), "Invalid block index!");

  Block* target = blocks[index];
  if (target) removeBlock(target);
}

void Chunck::removeBlockByPosition(Vec4* position) {
  Block* target = getBlockByPosition(position);
  if (target) removeBlock(target);
}

u8 Chunck::containsBlock(Vec4* offset) {
  return offset->x >= minOffset.x && offset->x < maxOffset.x &&
         offset->y >= minOffset.y && offset->y < maxOffset.y &&
         offset->z >= minOffset.z && offset->z < maxOffset.z;
}
