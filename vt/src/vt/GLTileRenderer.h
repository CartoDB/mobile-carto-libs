/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_VT_GLTILERENDERER_H_
#define _CARTO_VT_GLTILERENDERER_H_

#include "Color.h"
#include "ViewState.h"
#include "Tile.h"
#include "GLExtensions.h"
#include "GLShaderManager.h"

#include <memory>
#include <tuple>
#include <array>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <mutex>

#include <cglib/ray.h>

namespace carto { namespace vt {
    class GLTileRenderer final {
    public:
        // TODO: remove this constructor
        explicit GLTileRenderer(std::shared_ptr<std::mutex> mutex, std::shared_ptr<GLExtensions> glExtensions, float scale, bool useFBO, bool useDepth, bool useStencil);

        explicit GLTileRenderer(std::shared_ptr<std::mutex> mutex, std::shared_ptr<GLExtensions> glExtensions, float scale);

        void setViewState(const cglib::mat4x4<double>& projectionMatrix, const cglib::mat4x4<double>& cameraMatrix, float zoom, float aspectRatio, float resolution);
        void setLightDir(const cglib::vec3<float>& lightDir);
        void setInteractionMode(bool enabled);
        void setSubTileBlending(bool enabled);
        void setRenderSettings(bool useFBO, bool useDepth, bool useStencil, const Color& fboClearColor, float fboOpacity);
        void setBackground(const Color& color, std::shared_ptr<const BitmapPattern> pattern);
        void setVisibleTiles(const std::map<TileId, std::shared_ptr<const Tile>>& tiles, bool blend);
        std::vector<std::shared_ptr<TileLabel>> getVisibleLabels() const;

        // TODO: remove 3 methods
        void setFBOClearColor(const Color& clearColor);
        void setBackgroundColor(const Color& backgroundColor);
        void setBackgroundPattern(std::shared_ptr<const BitmapPattern> pattern);
        
        void initializeRenderer();
        void resetRenderer();
        void deinitializeRenderer();

        void startFrame(float dt);
        bool renderGeometry2D();
        bool renderGeometry3D();
        bool renderLabels(bool labels2D, bool labels3D);
        void endFrame();

        bool findGeometryIntersections(const cglib::ray3<double>& ray, std::vector<std::tuple<TileId, double, long long>>& results, float radius, bool geom2D, bool geom3D) const;
        bool findLabelIntersections(const cglib::ray3<double>& ray, std::vector<std::tuple<TileId, double, long long>>& results, float radius, bool labels2D, bool labels3D) const;
        bool findBitmapIntersections(const cglib::ray3<double>& ray, std::vector<std::tuple<TileId, double, TileBitmap, cglib::vec2<float>>>& results) const;

    private:
        using BitmapLabelMap = std::unordered_map<std::shared_ptr<const Bitmap>, std::vector<std::shared_ptr<TileLabel>>>;

        struct BlendNode {
            TileId tileId;
            std::shared_ptr<const Tile> tile;
            float blend;
            std::vector<std::shared_ptr<BlendNode>> childNodes;

            explicit BlendNode(const TileId& tileId, std::shared_ptr<const Tile> tile, float blend) : tileId(tileId), tile(std::move(tile)), blend(blend), childNodes() { }
        };

        struct RenderNode {
            TileId tileId;
            std::shared_ptr<const TileLayer> layer;
            float initialBlend;
            float blend;

            explicit RenderNode(const TileId& tileId, std::shared_ptr<const TileLayer> layer, float blend) : tileId(tileId), layer(std::move(layer)), initialBlend(blend), blend(blend) { }
        };

        struct LayerFBO {
            GLuint colorTexture;
            GLuint stencilRB;
            GLuint fbo;

            LayerFBO() : colorTexture(0), stencilRB(0), fbo(0) { }
        };

        struct ScreenFBO {
            GLuint colorTexture;
            GLuint depthStencilRB;
            GLuint fbo;
            std::vector<GLenum> depthStencilAttachments;

            ScreenFBO() : colorTexture(0), depthStencilRB(0), fbo(0), depthStencilAttachments() { }
        };

        struct TileVBO {
            GLuint vbo;

            TileVBO() : vbo(0) { }
        };

        struct ScreenVBO {
            GLuint vbo;

            ScreenVBO() : vbo(0) { }
        };

        struct CompiledBitmap {
            GLuint texture;

            CompiledBitmap() : texture(0) { }
        };

        struct CompiledGeometry {
            GLuint vertexGeometryVBO;
            GLuint indicesVBO;
            GLuint geometryVAO;

            CompiledGeometry() : vertexGeometryVBO(0), indicesVBO(0), geometryVAO(0) { }
        };

        struct CompiledLabelBatch {
            GLuint vertexGeometryVBO;
            GLuint vertexUVVBO;
            GLuint vertexAttribsVBO;
            GLuint vertexIndicesVBO;

            CompiledLabelBatch() : vertexGeometryVBO(0), vertexUVVBO(0), vertexAttribsVBO(0), vertexIndicesVBO(0) { }
        };

        struct LabelBatchParameters {
            constexpr static int MAX_PARAMETERS = 16;

            int parameterCount;
            float scale;
            std::array<cglib::vec4<float>, MAX_PARAMETERS> colorTable;
            std::array<float, MAX_PARAMETERS> widthTable;
            std::array<float, MAX_PARAMETERS> strokeWidthTable;

            LabelBatchParameters() : parameterCount(0), scale(0), colorTable(), widthTable(), strokeWidthTable() { }
        };

        struct LabelHash {
            std::size_t operator()(const std::pair<int, long long>& labelId) const {
                return labelId.first ^ static_cast<std::size_t>(labelId.second & 0xffffffff) ^ static_cast<std::size_t>(labelId.second >> 32);
            }
        };

        constexpr static float SDF_SHARPNESS_SCALE = 14.0f;
        constexpr static float HALO_RADIUS_SCALE = 2.5f; // the scaling factor for halo radius

        static cglib::mat4x4<double> calculateLocalViewMatrix(const cglib::mat4x4<double>& cameraMatrix);

        cglib::mat4x4<double> calculateTileMatrix(const TileId& tileId, float coordScale = 1.0f) const;
        cglib::mat3x3<double> calculateTileMatrix2D(const TileId& tileId, float coordScale = 1.0f) const;
        cglib::mat4x4<float> calculateTileMVPMatrix(const TileId& tileId, float coordScale = 1.0f) const;
        cglib::vec4<double> calculateTileOrigin(const TileId& tileId) const;
        cglib::bbox3<double> calculateTileBBox(const TileId& tileId) const;

        float calculateBlendNodeOpacity(const BlendNode& blendNode, float blend) const;
        
        void updateBlendNode(BlendNode& blendNode, float dBlend) const;
        bool buildRenderNodes(const BlendNode& blendNode, float blend, std::multimap<int, RenderNode>& renderNodeMap) const;
        void addRenderNode(RenderNode renderNode, std::multimap<int, RenderNode>& renderNodeMap) const;
        void updateLabels(const std::vector<std::shared_ptr<TileLabel>>& labels, float dOpacity) const;

        void setupPointCoordinateSystem(PointOrientation orientation, const TileId& tileId, float vertexScale, cglib::vec3<float>& xAxis, cglib::vec3<float>& yAxis) const;

        void findTileGeometryIntersections(const TileId& tileId, const std::shared_ptr<TileGeometry>& geometry, const cglib::ray3<double>& ray, float radius, std::vector<std::pair<double, long long>>& results) const;
        bool findLabelIntersection(const std::shared_ptr<TileLabel>& label, const cglib::ray3<double>& ray, float radius, double& result) const;

        cglib::vec3<float> decodeVertex(const std::shared_ptr<TileGeometry>& geometry, std::size_t index) const;
        cglib::vec3<float> decodePointOffset(const std::shared_ptr<TileGeometry>& geometry, std::size_t index, const cglib::vec3<float>& xAxis, const cglib::vec3<float>& yAxis, float radius) const;
        cglib::vec3<float> decodeLineOffset(const std::shared_ptr<TileGeometry>& geometry, std::size_t index, float radius) const;
        cglib::vec3<float> decodePolygon3DOffset(const std::shared_ptr<TileGeometry>& geometry, std::size_t index) const;

        bool renderBlendNodes2D(const std::vector<std::shared_ptr<BlendNode>>& blendNodes);
        bool renderBlendNodes3D(const std::vector<std::shared_ptr<BlendNode>>& blendNodes);
        bool renderLabels(const std::shared_ptr<const Bitmap>& bitmap, const std::vector<std::shared_ptr<TileLabel>>& labels);

        void blendScreenTexture(float opacity, GLuint texture);
        void blendTileTexture(const TileId& tileId, float opacity, GLuint texture);
        void renderTileMask(const TileId& tileId);
        void renderTileBackground(const TileId& tileId, float opacity);
        void renderTileBitmap(const TileId& tileId, const TileId& targetTileId, float blend, float opacity, const std::shared_ptr<TileBitmap>& bitmap);
        void renderTileGeometry(const TileId& tileId, const TileId& targetTileId, float blend, float opacity, const std::shared_ptr<TileGeometry>& geometry);
        void renderLabelBatch(const LabelBatchParameters& labelBatchParams, const std::shared_ptr<const Bitmap>& bitmap);
        void setBlendState(CompOp compOp);
        bool isEmptyBlendRequired(CompOp compOp) const;
        void checkGLError();

        GLuint createBuffer();
        void deleteBuffer(GLuint& buffer);
        GLuint createVertexArray();
        void deleteVertexArray(GLuint& vertexArray);
        GLuint createTexture();
        void deleteTexture(GLuint& texture);
        LayerFBO createLayerFBO(bool useStencil);
        void deleteLayerFBO(LayerFBO& layerFBO);
        ScreenFBO createScreenFBO(bool useDepth, bool useStencil);
        void deleteScreenFBO(ScreenFBO& screenFBO);
        TileVBO createTileVBO();
        void deleteTileVBO(TileVBO& tileVBO);
        ScreenVBO createScreenVBO();
        void deleteScreenVBO(ScreenVBO& screenVBO);

        bool _subTileBlending = false;
        bool _interactionMode = false;
        bool _useFBO = false;
        bool _useDepth = true;
        bool _useStencil = true;
        float _fboOpacity = 1.0f;
        Color _fboClearColor;
        Color _backgroundColor;
        std::shared_ptr<const BitmapPattern> _backgroundPattern;

        GLShaderManager::ShaderContext _patternTransformContext[2][2];
        GLShaderManager::ShaderContext _perspectiveAndDerivativesContext[2];
        GLShaderManager _shaderManager;

        std::vector<LayerFBO> _layerFBOs;
        ScreenFBO _screenFBO;
        ScreenFBO _overlayFBO;
        TileVBO _tileVBO;
        ScreenVBO _screenVBO;

        cglib::vec3<float> _lightDir;
        cglib::mat4x4<double> _projectionMatrix;
        cglib::mat4x4<double> _cameraMatrix;
        cglib::mat4x4<double> _cameraProjMatrix;
        cglib::frustum3<double> _frustum;
        cglib::mat4x4<double> _labelMatrix;
        ViewState _viewState;
        VertexArray<cglib::vec3<float>> _labelVertices;
        VertexArray<cglib::vec2<short>> _labelTexCoords;
        VertexArray<cglib::vec4<char>> _labelAttribs;
        VertexArray<unsigned short> _labelIndices;
        float _zoom = 0;
        float _halfResolution = 0;
        int _screenWidth = 0;
        int _screenHeight = 0;

        std::shared_ptr<std::vector<std::shared_ptr<BlendNode>>> _blendNodes;
        std::shared_ptr<std::vector<std::shared_ptr<BlendNode>>> _renderBlendNodes;
        std::array<std::shared_ptr<BitmapLabelMap>, 2> _bitmapLabelMap; // for 'ground' labels and for 'billboard' labels
        std::array<std::shared_ptr<BitmapLabelMap>, 2> _renderBitmapLabelMap;  // for 'ground' labels and for 'billboard' labels
        std::vector<std::shared_ptr<TileLabel>> _labels;
        std::unordered_map<std::pair<int, long long>, std::shared_ptr<TileLabel>, LabelHash> _labelMap;
        std::map<std::weak_ptr<const Bitmap>, CompiledBitmap, std::owner_less<std::weak_ptr<const Bitmap>>> _compiledBitmapMap;
        std::map<std::weak_ptr<const TileBitmap>, CompiledBitmap, std::owner_less<std::weak_ptr<const TileBitmap>>> _compiledTileBitmapMap;
        std::map<std::weak_ptr<const TileGeometry>, CompiledGeometry, std::owner_less<std::weak_ptr<const TileGeometry>>> _compiledTileGeometryMap;
        std::map<int, CompiledLabelBatch> _compiledLabelBatches;
        int _labelBatchCounter = 0;

        const float _scale;
        const std::shared_ptr<GLExtensions> _glExtensions;
        const std::shared_ptr<std::mutex> _mutex;
    };
} }

#endif
