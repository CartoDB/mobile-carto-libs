/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_VT_GLTILERENDERER_H_
#define _CARTO_VT_GLTILERENDERER_H_

#include "Bitmap.h"
#include "Color.h"
#include "ViewState.h"
#include "Label.h"
#include "Styles.h"
#include "Tile.h"
#include "TileId.h"
#include "TileTransformer.h"
#include "TileBitmap.h"
#include "TileBackground.h"
#include "TileBitmap.h"
#include "TileSurface.h"
#include "TileSurfaceBuilder.h"
#include "GLExtensions.h"

#include <memory>
#include <tuple>
#include <optional>
#include <array>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <utility>
#include <regex>
#include <mutex>

#include <cglib/ray.h>

namespace carto::vt {
    class LabelCuller;

    class GLTileRenderer final {
    public:
        struct LightingShader {
            bool perVertex;
            std::string shader;
            std::function<void(GLuint, const ViewState&)> setupFunc;

            explicit LightingShader(bool perVertex, std::string shader, std::function<void(GLuint, const ViewState&)> setupFunc) : perVertex(perVertex), shader(std::move(shader)), setupFunc(std::move(setupFunc)) { }
        };

        struct GeometryIntersectionInfo {
            TileId tileId;
            int layerIndex;
            long long featureId;
            std::size_t rayIndex;
            double rayT;

            explicit GeometryIntersectionInfo(const TileId& tileId, int layerIndex, long long featureId, std::size_t rayIndex, double rayT) : tileId(tileId), layerIndex(layerIndex), featureId(featureId), rayIndex(rayIndex), rayT(rayT) { }
        };

        struct BitmapIntersectionInfo {
            TileId tileId;
            int layerIndex;
            std::shared_ptr<const TileBitmap> bitmap;
            cglib::vec2<float> uv;
            std::size_t rayIndex;
            double rayT;

            explicit BitmapIntersectionInfo(const TileId& tileId, int layerIndex, std::shared_ptr<const TileBitmap> bitmap, const cglib::vec2<float>& uv, std::size_t rayIndex, double rayT) : tileId(tileId), layerIndex(layerIndex), bitmap(bitmap), uv(uv), rayIndex(rayIndex), rayT(rayT) { }
        };
        
        explicit GLTileRenderer(std::shared_ptr<GLExtensions> glExtensions, std::shared_ptr<const TileTransformer> transformer, float scale);

        void setLightingShader2D(const std::optional<LightingShader>& lightingShader2D);
        void setLightingShader3D(const std::optional<LightingShader>& lightingShader3D);
        void setLightingShaderNormalMap(const std::optional<LightingShader>& lightingShaderNormalMap);
        
        void setInteractionMode(bool enabled);
        void setLayerBlendingSpeed(float speed);
        void setLabelBlendingSpeed(float speed);
        void setRasterFilterMode(RasterFilterMode filterMode);
        void setRendererLayerFilter(const std::optional<std::regex>& filter);
        void setClickHandlerLayerFilter(const std::optional<std::regex>& filter);
        void setViewState(const ViewState& viewState);
        void setVisibleTiles(const std::map<TileId, std::shared_ptr<const Tile>>& tiles);
        void teleportVisibleTiles(int dx, int dy);

        void initializeRenderer();
        void resetRenderer();
        void deinitializeRenderer();

        bool startFrame(float dt);
        void renderGeometry(bool geom2D, bool geom3D);
        void renderLabels(bool labels2D, bool labels3D);
        bool endFrame();

        void cullLabels(LabelCuller& culler);

        bool findBitmapIntersections(const std::vector<cglib::ray3<double>>& rays, std::vector<BitmapIntersectionInfo>& results) const;
        bool findGeometryIntersections(const std::vector<cglib::ray3<double>>& rays, float pointBuffer, float lineBuffer, bool geom2D, bool geom3D, std::vector<GeometryIntersectionInfo>& results) const;
        bool findLabelIntersections(const std::vector<cglib::ray3<double>>& rays, float buffer, bool labels2D, bool labels3D, std::vector<GeometryIntersectionInfo>& results) const;

    private:
        using GlobalIdLabelMap = std::unordered_map<long long, std::shared_ptr<Label>>;
        using BitmapLabelMap = std::unordered_map<std::shared_ptr<const Bitmap>, std::vector<std::shared_ptr<Label>>>;

        enum class LightingMode {
            NONE,
            GEOMETRY2D,
            GEOMETRY3D,
            NORMALMAP
        };

        struct RenderTileLayer {
            TileId targetTileId = TileId(-1, -1, -1);
            TileId sourceTileId = TileId(-1, -1, -1);
            std::shared_ptr<const TileLayer> layer;
            float tileSize = 0.0f;
            bool active = false;
            float blend = 0.0f;
        };

        struct RenderTile {
            TileId targetTileId = TileId(-1, -1, -1);
            std::shared_ptr<const Tile> tile;
            std::multimap<int, RenderTileLayer> renderLayers;
            bool visible = false;
        };

        struct FrameBuffer {
            GLuint colorTexture;
            std::vector<GLuint> depthStencilRBs;
            std::vector<GLenum> depthStencilAttachments;
            GLuint fbo;

            FrameBuffer() : colorTexture(0), depthStencilRBs(), depthStencilAttachments(), fbo(0) { }
        };

        struct ShaderProgram {
            GLuint program;
            std::vector<GLuint> uniforms;
            std::vector<GLuint> attribs;

            ShaderProgram() : program(0), uniforms(), attribs() { }
        };

        struct CompiledBitmap {
            GLuint texture;

            CompiledBitmap() : texture(0) { }
        };

        struct CompiledQuad {
            GLuint vbo;

            CompiledQuad() : vbo(0) { }
        };

        struct CompiledSurface {
            GLuint vertexGeometryVBO;
            GLuint indicesVBO;

            CompiledSurface() : vertexGeometryVBO(0), indicesVBO(0) { }
        };

        struct CompiledGeometry {
            GLuint vertexGeometryVBO;
            GLuint indicesVBO;
            GLuint geometryVAO;
            mutable bool geometryVAOInitialized;

            CompiledGeometry() : vertexGeometryVBO(0), indicesVBO(0), geometryVAO(0), geometryVAOInitialized(false) { }
        };

        struct CompiledLabelBatch {
            GLuint verticesVBO;
            GLuint normalsVBO;
            GLuint texCoordsVBO;
            GLuint attribsVBO;
            GLuint indicesVBO;

            CompiledLabelBatch() : verticesVBO(0), normalsVBO(0), texCoordsVBO(0), attribsVBO(0), indicesVBO(0) { }
        };

        struct LabelBatchParameters {
            static constexpr int MAX_PARAMETERS = 16;

            int labelCount;
            int parameterCount;
            float scale;
            cglib::mat4x4<double> labelMatrix;
            std::array<cglib::vec4<float>, MAX_PARAMETERS> colorTable;
            std::array<float, MAX_PARAMETERS> widthTable;
            std::array<float, MAX_PARAMETERS> strokeWidthTable;

            LabelBatchParameters() : labelCount(0), parameterCount(0), scale(0), labelMatrix(cglib::mat4x4<double>::identity()), colorTable(), widthTable(), strokeWidthTable() { }
        };

        static constexpr float HALO_RADIUS_SCALE = 2.5f; // the scaling factor for halo radius
        static constexpr float STROKE_UV_SCALE = 2.857f; // stroked line UV scale factor
        static constexpr float POLYGON3D_HEIGHT_SCALE = 10018754.17f; // scaling factor for zoom 0 heights
        static constexpr float ALPHA_HIT_THRESHOLD = 0.05f; // threshold value for 'transparent' pixel alphas

        bool isTileVisible(const TileId& tileId) const;
        bool isEmptyBlendRequired(CompOp compOp) const;

        cglib::mat4x4<double> calculateTileMatrix(const TileId& tileId, float coordScale = 1.0f) const;
        cglib::mat3x3<double> calculateTileMatrix2D(const TileId& tileId, float coordScale = 1.0f) const;
        cglib::mat4x4<float> calculateTileMVPMatrix(const TileId& tileId, float coordScale = 1.0f) const;

        bool testLayerFilter(const std::string& layerName, const std::optional<std::regex>& filter) const;
        bool testIntersectionOpacity(const std::shared_ptr<const BitmapPattern>& pattern, const cglib::vec2<float>& uvp, const cglib::vec2<float>& uv0, const cglib::vec2<float>& uv1) const;

        void buildTileSurfaces(const std::set<TileId>& tileIds);

        void buildRenderTiles(const std::map<TileId, std::shared_ptr<const Tile>>& tiles);
        void initializeRenderTile(TileId targetTileId, RenderTile& renderTile, const std::shared_ptr<const Tile>& tile, const std::vector<RenderTile>& existingRenderTiles) const;
        void mergeExistingRenderTile(TileId targetTileId, const RenderTile& existingRenderTile, std::vector<RenderTile>& renderTiles, int depth) const;
        bool updateRenderTile(RenderTile& renderTile, float dBlend) const;

        void buildLabelMaps(const std::vector<std::shared_ptr<const Tile>>& labelTiles);
        bool updateLabel(const std::shared_ptr<Label>& label, float dOpacity) const;

        void findTileGeometryIntersections(const TileId& tileId, const std::shared_ptr<const TileGeometry>& geometry, const std::vector<cglib::ray3<double>>& rays, float tileSize, float pointBuffer, float lineBuffer, float heightScale, std::vector<GeometryIntersectionInfo>& results) const;
        void findLabelIntersections(const std::shared_ptr<Label>& label, const std::vector<cglib::ray3<double>>& rays, float buffer, std::vector<GeometryIntersectionInfo>& results) const;
        void findTileBitmapIntersections(const TileId& tileId, const std::shared_ptr<const TileBitmap>& bitmap, const std::shared_ptr<const TileSurface>& tileSurface, const std::vector<cglib::ray3<double>>& rays, float tileSize, std::vector<BitmapIntersectionInfo>& results) const;

        void renderGeometry2D(const std::vector<RenderTile>& renderTiles, GLint stencilBits);
        void renderGeometry3D(const std::vector<RenderTile>& renderTiles);
        void renderLabels(const std::vector<std::shared_ptr<Label>>& labels, const std::shared_ptr<const Bitmap>& bitmap);

        void setCompOp(CompOp compOp);
        void blendScreenTexture(float opacity, GLuint texture);
        void renderTileMask(const TileId& tileId);
        void renderTileBackground(const TileId& tileId, float blend, float opacity, float tileSize, const std::shared_ptr<TileBackground>& background);
        void renderTileBitmap(const TileId& sourceTileId, const TileId& targetTileId, float blend, float opacity, const std::shared_ptr<TileBitmap>& bitmap);
        void renderTileGeometry(const TileId& sourceTileId, const TileId& targetTileId, float blend, float opacity, float tileSize, const std::shared_ptr<TileGeometry>& geometry);
        void renderLabelBatch(const LabelBatchParameters& labelBatchParams, const std::shared_ptr<const Bitmap>& bitmap);

        const CompiledBitmap& buildCompiledBitmap(const std::shared_ptr<const Bitmap>& bitmap, bool genMipmaps);
        const CompiledBitmap& buildCompiledTileBitmap(const std::shared_ptr<TileBitmap>& tileBitmap);
        const CompiledGeometry& buildCompiledTileGeometry(const std::shared_ptr<TileGeometry>& tileGeometry);
        const ShaderProgram& buildShaderProgram(const std::string& id, const std::string& vsh, const std::string& fsh, LightingMode lightingMode, RasterFilterMode filterMode, unsigned int flags);
        const std::vector<std::shared_ptr<TileSurface>>& buildCompiledTileSurfaces(const TileId& tileId);

        void createShaderProgram(ShaderProgram& shaderProgram, const std::string& vsh, const std::string& fsh, const std::set<std::string>& defs, const std::map<std::string, int>& uniformMap, const std::map<std::string, int>& attribMap);
        void deleteShaderProgram(ShaderProgram& shaderProgram);
        void createFrameBuffer(FrameBuffer& frameBuffer, bool useColor, bool useDepth, bool useStencil);
        void deleteFrameBuffer(FrameBuffer& frameBuffer);
        void createCompiledBitmap(CompiledBitmap& compiledBitmap);
        void deleteCompiledBitmap(CompiledBitmap& compiledBitmap);
        void createCompiledQuad(CompiledQuad& compiledQuad);
        void deleteCompiledQuad(CompiledQuad& compiledQuad);
        void createCompiledSurface(CompiledSurface& compiledSurface);
        void deleteCompiledSurface(CompiledSurface& compiledSurface);
        void createCompiledGeometry(CompiledGeometry& compiledGeometry);
        void deleteCompiledGeometry(CompiledGeometry& compiledGeometry);
        void createCompiledLabelBatch(CompiledLabelBatch& compiledLabelBatch);
        void deleteCompiledLabelBatch(CompiledLabelBatch& compiledLabelBatch);

        std::optional<LightingShader> _lightingShader2D;
        std::optional<LightingShader> _lightingShader3D;
        std::optional<LightingShader> _lightingShaderNormalMap;
        TileSurfaceBuilder _tileSurfaceBuilder;

        FrameBuffer _overlayBuffer2D;
        FrameBuffer _overlayBuffer3D;
        CompiledQuad _screenQuad;

        ViewState _viewState;
        cglib::mat4x4<double> _cameraProjMatrix = cglib::mat4x4<double>::identity();
        float _fullResolution = 0;
        float _halfResolution = 0;
        int _screenWidth = 0;
        int _screenHeight = 0;
        cglib::vec3<double> _tileSurfaceBuilderOrigin = cglib::vec3<double>(0, 0, 0);
        std::set<TileId> _tileSurfaceBuilderOriginTileIds;

        bool _interactionMode = false;
        float _layerBlendingSpeed = 1.0f;
        float _labelBlendingSpeed = 1.0f;
        RasterFilterMode _rasterFilterMode = RasterFilterMode::BILINEAR;
        std::optional<std::regex> _rendererLayerFilter;
        std::optional<std::regex> _clickHandlerLayerFilter;

        std::shared_ptr<std::vector<RenderTile>> _renderTiles;
        std::shared_ptr<std::vector<RenderTile>> _visibleRenderTiles;
        std::array<std::shared_ptr<BitmapLabelMap>, 2> _bitmapLabelMap; // for 'ground' labels and for 'billboard' labels
        std::array<std::shared_ptr<BitmapLabelMap>, 2> _visibleBitmapLabelMap;  // for 'ground' labels and for 'billboard' labels
        std::vector<std::shared_ptr<Label>> _labels;
        std::map<int, GlobalIdLabelMap> _layerLabelMap;
        std::map<TileId, std::vector<std::shared_ptr<TileSurface>>> _tileSurfaceMap;
        std::map<std::string, ShaderProgram> _shaderProgramMap;
        std::map<std::weak_ptr<const Bitmap>, CompiledBitmap, std::owner_less<std::weak_ptr<const Bitmap>>> _compiledBitmapMap;
        std::map<std::weak_ptr<const TileBitmap>, CompiledBitmap, std::owner_less<std::weak_ptr<const TileBitmap>>> _compiledTileBitmapMap;
        std::map<std::weak_ptr<const TileGeometry>, CompiledGeometry, std::owner_less<std::weak_ptr<const TileGeometry>>> _compiledTileGeometryMap;
        std::map<std::weak_ptr<const TileSurface>, CompiledSurface, std::owner_less<std::weak_ptr<const TileSurface>>> _compiledTileSurfaceMap;
        std::map<int, CompiledLabelBatch> _compiledLabelBatches;
        int _labelBatchCounter = 0;

        VertexArray<cglib::vec3<float>> _labelVertices;
        VertexArray<cglib::vec3<float>> _labelNormals;
        VertexArray<cglib::vec2<std::int16_t>> _labelTexCoords;
        VertexArray<cglib::vec4<std::int8_t>> _labelAttribs;
        VertexArray<std::uint16_t> _labelIndices;

        const std::shared_ptr<GLExtensions> _glExtensions;
        const std::shared_ptr<const TileTransformer> _transformer;
        const float _scale;

        mutable std::mutex _mutex;
    };
}

#endif
