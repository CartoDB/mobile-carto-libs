#include "TileLayerBuilder.h"
#include "TextFormatter.h"
#include "Color.h"

#include <utility>
#include <algorithm>
#include <iterator>

#include <boost/math/constants/constants.hpp>

#include <tesselator.h>

namespace carto { namespace vt {
    TileLayerBuilder::TileLayerBuilder(const TileId& tileId, float tileSize, float geomScale) :
        _tileId(tileId), _tileSize(tileSize), _geomScale(geomScale), _clipBox(Vertex(-0.1f, -0.1f), Vertex(1.1f, 1.1f))
    {
        _vertices.reserve(RESERVED_VERTICES);
        _texCoords.reserve(RESERVED_VERTICES);
        _binormals.reserve(RESERVED_VERTICES);
        _heights.reserve(RESERVED_VERTICES);
        _attribs.reserve(RESERVED_VERTICES);
        _indices.reserve(RESERVED_VERTICES);
        _ids.reserve(RESERVED_VERTICES);
    }

    void TileLayerBuilder::setClipBox(const cglib::bbox2<float>& clipBox) {
        _clipBox = clipBox;
    }

    void TileLayerBuilder::addBitmap(const std::shared_ptr<TileBitmap>& bitmap) {
        _bitmapList.push_back(bitmap);
    }

    void TileLayerBuilder::addPoints(const std::function<bool(long long& id, Vertex& vertex)>& generator, const PointStyle& style, const std::shared_ptr<GlyphMap>& glyphMap) {
        if (style.sizeFunc == FloatFunction(0) || !style.pointImage) {
            return;
        }
        
        long long id = 0;
        Vertex vertex(0, 0);
        if (!generator(id, vertex)) {
            return;
        }
        boost::optional<cglib::mat3x3<float>> transform = flipTransform(style.transform);

        if (_builderParameters.type != TileGeometry::Type::POINT || _builderParameters.glyphMap != glyphMap || _styleParameters.transform != transform || _styleParameters.compOp != style.compOp || _styleParameters.pointOrientation != style.orientation || _styleParameters.parameterCount >= TileGeometry::StyleParameters::MAX_PARAMETERS) {
            appendGeometry();
        }
        _builderParameters.type = TileGeometry::Type::POINT;
        _builderParameters.glyphMap = glyphMap;
        _styleParameters.transform = transform;
        _styleParameters.compOp = style.compOp;
        _styleParameters.pointOrientation = style.orientation;
        GlyphMap::GlyphId glyphId = glyphMap->loadBitmapGlyph(style.pointImage->bitmap, style.pointImage->sdfMode);
        int styleIndex = _styleParameters.parameterCount;
        while (--styleIndex >= 0) {
            if (_styleParameters.colorFuncs[styleIndex] == style.colorFunc && _styleParameters.widthFuncs[styleIndex] == style.sizeFunc && _styleParameters.strokeWidthFuncs[styleIndex] == FloatFunction(0)) {
                break;
            }
        }
        if (styleIndex < 0) {
            styleIndex = _styleParameters.parameterCount++;
            _styleParameters.colorFuncs[styleIndex] = style.colorFunc;
            _styleParameters.widthFuncs[styleIndex] = style.sizeFunc;
            _styleParameters.strokeWidthFuncs[styleIndex] = FloatFunction(0);
        }
        
        do {
            std::size_t i0 = _indices.size();
            cglib::vec2<float> pen(0, 0);
            const GlyphMap::Glyph* glyph = glyphMap->getGlyph(glyphId);
            if (glyph) {
                pen = -cglib::vec2<float>(glyph->width, glyph->height) * 0.5f;
            }
            tesselateGlyph(vertex, static_cast<char>(styleIndex), pen * style.pointImage->scale, cglib::vec2<float>(glyph->width, glyph->height) * style.pointImage->scale, glyph);
            _ids.fill(id, _indices.size() - i0);
        } while (generator(id, vertex));
    }

    void TileLayerBuilder::addTexts(const std::function<bool(long long& id, Vertex& vertex, std::string& text)>& generator, const TextStyle& style, const TextFormatter& formatter) {
        if (style.sizeFunc == FloatFunction(0) && !style.backgroundImage) {
            return;
        }
        
        long long id = 0;
        Vertex vertex(0, 0);
        std::string text;
        if (!generator(id, vertex, text)) {
            return;
        }
        
        const std::shared_ptr<Font>& font = formatter.getFont();
        boost::optional<cglib::mat3x3<float>> transform = flipTransform(style.transform);

        if (_builderParameters.type != TileGeometry::Type::POINT || _builderParameters.glyphMap != font->getGlyphMap() || _styleParameters.transform != transform || _styleParameters.compOp != style.compOp || _styleParameters.pointOrientation != style.orientation || _styleParameters.parameterCount + 2 > TileGeometry::StyleParameters::MAX_PARAMETERS) {
            appendGeometry();
        }
        _builderParameters.type = TileGeometry::Type::POINT;
        _builderParameters.glyphMap = font->getGlyphMap();
        _styleParameters.transform = transform;
        _styleParameters.compOp = style.compOp;
        _styleParameters.pointOrientation = style.orientation;
        int styleIndex = _styleParameters.parameterCount;
        while (--styleIndex >= 0) {
            if (_styleParameters.colorFuncs[styleIndex] == style.colorFunc && _styleParameters.widthFuncs[styleIndex] == style.sizeFunc && _styleParameters.strokeWidthFuncs[styleIndex] == FloatFunction(0)) {
                break;
            }
        }
        if (styleIndex < 0) {
            styleIndex = _styleParameters.parameterCount++;
            _styleParameters.colorFuncs[styleIndex] = style.colorFunc;
            _styleParameters.widthFuncs[styleIndex] = style.sizeFunc;
            _styleParameters.strokeWidthFuncs[styleIndex] = FloatFunction(0);
        }

        int haloStyleIndex = -1;
        if (style.haloRadiusFunc != FloatFunction(0)) {
            for (haloStyleIndex = _styleParameters.parameterCount; --haloStyleIndex >= 0; ) {
                if (_styleParameters.colorFuncs[haloStyleIndex] == style.haloColorFunc && _styleParameters.widthFuncs[haloStyleIndex] == style.sizeFunc && _styleParameters.strokeWidthFuncs[haloStyleIndex] == style.haloRadiusFunc) {
                    break;
                }
            }
            if (haloStyleIndex < 0) {
                haloStyleIndex = _styleParameters.parameterCount++;
                _styleParameters.colorFuncs[haloStyleIndex] = style.haloColorFunc;
                _styleParameters.widthFuncs[haloStyleIndex] = style.sizeFunc;
                _styleParameters.strokeWidthFuncs[haloStyleIndex] = style.haloRadiusFunc;
            }
        }

        do {
            std::size_t i0 = _indices.size();
            std::size_t i1 = _binormals.size();
            std::vector<Font::Glyph> glyphs = formatter.format(text, 1.0f);
            if (style.backgroundImage) {
                const GlyphMap::Glyph* baseGlyph = font->getGlyphMap()->getGlyph(font->getGlyphMap()->loadBitmapGlyph(style.backgroundImage->bitmap, style.backgroundImage->sdfMode));
                if (baseGlyph) {
                    float scale = style.backgroundImage->scale / formatter.getFontSize();
                    Font::Glyph glyph(0, *baseGlyph, cglib::vec2<float>(baseGlyph->width, baseGlyph->height) * (style.backgroundScale * scale), style.backgroundOffset * scale, cglib::vec2<float>(0, 0));
                    tesselateGlyph(vertex, styleIndex, glyph.offset * style.backgroundImage->scale, glyph.size * style.backgroundImage->scale, &glyph.baseGlyph);
                }
            }

            for (int pass = (haloStyleIndex >= 0 ? 0 : 1); pass < 2; pass++) {
                cglib::vec2<float> pen(0, 0);
                for (Font::Glyph& glyph : glyphs) {
                    if (glyph.codePoint == Font::CR_CODEPOINT) {
                        pen = cglib::vec2<float>(0, 0);
                    }
                    else {
                        tesselateGlyph(vertex, static_cast<char>(pass == 0 ? haloStyleIndex : styleIndex), pen + glyph.offset, glyph.size, &glyph.baseGlyph);
                    }

                    pen += glyph.advance;
                }
            }
            _ids.fill(id, _indices.size() - i0);

            if (style.angle != 0) {
                cglib::mat3x3<float> transform = cglib::rotate3_matrix(cglib::vec3<float>(0, 0, 1), style.angle * boost::math::constants::pi<float>() / 180.0f);
                for (std::size_t i = i1; i < _binormals.size(); i++) {
                    _binormals[i] = cglib::transform_vector(_binormals[i], transform); // NOTE: no need to use special inv/transposed matrix
                }
            }
        } while (generator(id, vertex, text));
    }

    void TileLayerBuilder::addLines(const std::function<bool(long long& id, Vertices& vertices)>& generator, const LineStyle& style, const std::shared_ptr<StrokeMap>& strokeMap) {
        if (style.widthFunc == FloatFunction(0)) {
            return;
        }
        
        long long id = 0;
        Vertices vertices;
        if (!generator(id, vertices)) {
            return;
        }

        if ((_builderParameters.strokeMap && _builderParameters.strokeMap != strokeMap) || _styleParameters.transform != style.transform || _styleParameters.compOp != style.compOp || _styleParameters.parameterCount >= TileGeometry::StyleParameters::MAX_PARAMETERS) {
            appendGeometry();
        }
        else if (!(_builderParameters.type == TileGeometry::Type::LINE || (_builderParameters.type == TileGeometry::Type::POLYGON && !_styleParameters.pattern && !_styleParameters.transform))) { // we can use also line drawing shader but ONLY if pattern/transform is not used for polygons (pattern can be used for lines)
            appendGeometry();
        }
        _builderParameters.type = TileGeometry::Type::LINE;
        _builderParameters.strokeMap = strokeMap;
        _styleParameters.transform = style.transform;
        _styleParameters.compOp = style.compOp;
        StrokeMap::StrokeId strokeId = (style.strokePattern ? strokeMap->loadBitmapPattern(style.strokePattern) : 0);
        const StrokeMap::Stroke* stroke = strokeMap->getStroke(strokeId);
        int styleIndex = _styleParameters.parameterCount;
        while (--styleIndex >= 0) {
            if (_styleParameters.colorFuncs[styleIndex] == style.colorFunc && _styleParameters.widthFuncs[styleIndex] == style.widthFunc && _builderParameters.lineStrokeIds[styleIndex] == strokeId) {
                break;
            }
        }
        if (styleIndex < 0) {
            styleIndex = _styleParameters.parameterCount++;
            _styleParameters.colorFuncs[styleIndex] = style.colorFunc;
            _styleParameters.widthFuncs[styleIndex] = style.widthFunc;
            _builderParameters.lineStrokeIds[styleIndex] = strokeId;
        }
        
        do {
            std::size_t i0 = _indices.size();
            _binormals.fill(cglib::vec2<float>(0, 0), _vertices.size() - _binormals.size()); // needed if previously only polygons were used
            tesselateLine(vertices, static_cast<char>(styleIndex), stroke, style);
            _ids.fill(id, _indices.size() - i0);
        } while (generator(id, vertices));
    }

    void TileLayerBuilder::addPolygons(const std::function<bool(long long& id, VerticesList& verticesList)>& generator, const PolygonStyle& style) {
        long long id = 0;
        VerticesList verticesList;
        if (!generator(id, verticesList)) {
            return;
        }

        TileGeometry::Type type = TileGeometry::Type::POLYGON;
        if (_styleParameters.pattern != style.pattern || _styleParameters.transform != style.transform || _styleParameters.compOp != style.compOp || _styleParameters.parameterCount >= TileGeometry::StyleParameters::MAX_PARAMETERS) {
            appendGeometry();
        }
        else if (!(_builderParameters.type == TileGeometry::Type::POLYGON || (_builderParameters.type == TileGeometry::Type::LINE && !style.pattern && !style.transform))) { // we can use also line drawing shader but ONLY if pattern/transform is not used for polygons (pattern can be used for lines)
            appendGeometry();
        }
        else {
            type = _builderParameters.type;
        }
        _builderParameters.type = type;
        _styleParameters.pattern = style.pattern;
        _styleParameters.transform = style.transform;
        _styleParameters.compOp = style.compOp;
        int styleIndex = _styleParameters.parameterCount;
        while (--styleIndex >= 0) {
            if (_styleParameters.colorFuncs[styleIndex] == style.colorFunc && _styleParameters.widthFuncs[styleIndex] == FloatFunction(0) && _builderParameters.lineStrokeIds[styleIndex] == 0) {
                break;
            }
        }
        if (styleIndex < 0) {
            styleIndex = _styleParameters.parameterCount++;
            _styleParameters.colorFuncs[styleIndex] = style.colorFunc;
            _styleParameters.widthFuncs[styleIndex] = FloatFunction(0); // fill width information when we need to use line shader with polygons
            _builderParameters.lineStrokeIds[styleIndex] = 0; // fill stroke information when we need to use line shader with polygons
        }

        do {
            std::size_t i0 = _ids.size();
            tesselatePolygon(verticesList, static_cast<char>(styleIndex), style);
            _ids.fill(id, _indices.size() - i0);
            if (type == TileGeometry::Type::LINE) {
                _binormals.fill(cglib::vec2<float>(0, 0), _vertices.size() - _binormals.size()); // use zero binormals if using 'lines'
            }
        } while (generator(id, verticesList));
    }

    void TileLayerBuilder::addPolygons3D(const std::function<bool(long long& id, VerticesList& verticesList)>& generator, float minHeight, float maxHeight, const Polygon3DStyle& style) {
        if (minHeight > maxHeight) {
            return;
        }

        long long id = 0;
        VerticesList verticesList;
        if (!generator(id, verticesList)) {
            return;
        }

        if (_builderParameters.type != TileGeometry::Type::POLYGON3D || _styleParameters.transform != style.transform || _styleParameters.parameterCount >= TileGeometry::StyleParameters::MAX_PARAMETERS) {
            appendGeometry();
        }
        _builderParameters.type = TileGeometry::Type::POLYGON3D;
        _styleParameters.transform = style.transform;
        int styleIndex = _styleParameters.parameterCount;
        while (--styleIndex >= 0) {
            if (_styleParameters.colorFuncs[styleIndex] == style.colorFunc) {
                break;
            }
        }
        if (styleIndex < 0) {
            styleIndex = _styleParameters.parameterCount++;
            _styleParameters.colorFuncs[styleIndex] = style.colorFunc;
        }

        do {
            std::size_t i0 = _ids.size();
            tesselatePolygon3D(verticesList, minHeight, maxHeight, static_cast<char>(styleIndex), style);
            _ids.fill(id, _indices.size() - i0);
        } while (generator(id, verticesList));
    }

    void TileLayerBuilder::addBitmapLabels(const std::function<bool(long long& id, BitmapLabelInfo& labelInfo)>& generator, const BitmapLabelStyle& style, const std::shared_ptr<GlyphMap>& glyphMap) {
        if (style.sizeFunc == FloatFunction(0) || !style.image) {
            return;
        }

        boost::optional<cglib::mat3x3<float>> transform = flipTransform(style.transform);

        const GlyphMap::Glyph* baseGlyph = glyphMap->getGlyph(glyphMap->loadBitmapGlyph(style.image->bitmap, style.image->sdfMode));
        if (!baseGlyph) {
            return;
        }
        std::vector<Font::Glyph> bitmapGlyphs = {
            Font::Glyph(Font::CR_CODEPOINT, GlyphMap::Glyph(false, 0, 0, 0, 0, cglib::vec2<float>(0, 0)), cglib::vec2<float>(0, 0), cglib::vec2<float>(0, 0), -cglib::vec2<float>(style.image->bitmap->width, style.image->bitmap->height) * (style.image->scale * 0.5f)),
            Font::Glyph(0, *baseGlyph, cglib::vec2<float>(baseGlyph->width, baseGlyph->height) * style.image->scale, cglib::vec2<float>(0, 0), cglib::vec2<float>(0, 0))
        };

        float scale = 1.0f / _tileSize;
        if (!_labelStyle || _labelStyle->orientation != style.orientation || _labelStyle->colorFunc != style.colorFunc || _labelStyle->sizeFunc != style.sizeFunc || _labelStyle->haloColorFunc != ColorFunction() || _labelStyle->haloRadiusFunc != FloatFunction() || _labelStyle->scale != scale || _labelStyle->ascent != 0.0f || _labelStyle->descent != 0.0f || _labelStyle->transform != transform || _labelStyle->glyphMap != glyphMap) {
            _labelStyle = std::make_shared<TileLabel::LabelStyle>(style.orientation, style.colorFunc, style.sizeFunc, ColorFunction(), FloatFunction(), 1.0f / _tileSize, 0.0f, 0.0f, transform, glyphMap);
        }
        
        while (true) {
            long long id = 0;
            BitmapLabelInfo labelInfo;
            if (!generator(id, labelInfo)) {
                break;
            }

            boost::optional<cglib::vec3<double>> labelPosition;
            std::vector<cglib::vec3<double>> labelVertices;
            if (auto vertex = boost::get<Vertex>(&labelInfo.position)) {
                labelPosition = cglib::vec3<double>((*vertex)(0), (*vertex)(1), 0);
            }
            else if (auto vertices = boost::get<Vertices>(&labelInfo.position)) {
                labelVertices.resize(vertices->size());
                for (std::size_t i = 0; i < vertices->size(); i++) {
                    labelVertices[i] = cglib::vec3<double>((*vertices)[i](0), (*vertices)[i](1), 0);
                }
            }

            auto bitmapLabel = std::make_shared<TileLabel>(_tileId, id, labelInfo.id, labelInfo.groupId, std::move(bitmapGlyphs), std::move(labelPosition), std::move(labelVertices), _labelStyle);
            bitmapLabel->setMinimumGroupDistance(_tileSize * labelInfo.minimumGroupDistance);
            _labelList.push_back(std::move(bitmapLabel));
        }
    }

    void TileLayerBuilder::addTextLabels(const std::function<bool(long long& id, TextLabelInfo& labelInfo)>& generator, const TextLabelStyle& style, const TextFormatter& formatter) {
        if (style.sizeFunc == FloatFunction(0) && !style.backgroundImage) {
            return;
        }
        
        boost::optional<cglib::mat3x3<float>> transform;
        if (style.angle != 0) {
            transform = cglib::rotate3_matrix(cglib::vec3<float>(0, 0, 1), style.angle * boost::math::constants::pi<float>() / 180.0f);
        }

        const std::shared_ptr<Font>& font = formatter.getFont();
        float scale = 1.0f / _tileSize;
        Font::Metrics metrics = formatter.getFont()->getMetrics(1.0f);
        if (!_labelStyle || _labelStyle->orientation != style.orientation || _labelStyle->colorFunc != style.colorFunc || _labelStyle->sizeFunc != style.sizeFunc || _labelStyle->haloColorFunc != style.haloColorFunc || _labelStyle->haloRadiusFunc != style.haloRadiusFunc || _labelStyle->scale != scale || _labelStyle->ascent != metrics.ascent || _labelStyle->descent != metrics.descent || _labelStyle->transform != transform || _labelStyle->glyphMap != font->getGlyphMap()) {
            _labelStyle = std::make_shared<TileLabel::LabelStyle>(style.orientation, style.colorFunc, style.sizeFunc, style.haloColorFunc, style.haloRadiusFunc, scale, metrics.ascent, metrics.descent, transform, font->getGlyphMap());
        }

        while (true) {
            long long id = 0;
            TextLabelInfo labelInfo;
            if (!generator(id, labelInfo)) {
                break;
            }

            if (!labelInfo.text.empty() || style.backgroundImage) {
                std::vector<Font::Glyph> glyphs = formatter.format(labelInfo.text, 1.0f);
                if (style.backgroundImage) {
                    const GlyphMap::Glyph* baseGlyph = font->getGlyphMap()->getGlyph(font->getGlyphMap()->loadBitmapGlyph(style.backgroundImage->bitmap, style.backgroundImage->sdfMode));
                    if (baseGlyph) {
                        float scale = style.backgroundImage->scale / formatter.getFontSize();
                        glyphs.insert(glyphs.begin(), Font::Glyph(0, *baseGlyph, cglib::vec2<float>(baseGlyph->width, baseGlyph->height) * (style.backgroundScale * scale), style.backgroundOffset * scale, cglib::vec2<float>(baseGlyph->width, 0) * scale));
                    }
                }

                boost::optional<cglib::vec3<double>> labelPosition;
                if (auto vertex = boost::get<Vertex>(&labelInfo.position)) {
                    labelPosition = cglib::vec3<double>((*vertex)(0), (*vertex)(1), 0);
                }
                std::vector<cglib::vec3<double>> labelVertices;
                labelVertices.reserve(labelInfo.vertices.size());
                for (const Vertex& vertex : labelInfo.vertices) {
                    labelVertices.emplace_back(vertex(0), vertex(1), 0);
                }

                auto textLabel = std::make_shared<TileLabel>(_tileId, id, labelInfo.id, labelInfo.groupId, std::move(glyphs), std::move(labelPosition), std::move(labelVertices), _labelStyle);
                textLabel->setMinimumGroupDistance(_tileSize * labelInfo.minimumGroupDistance);
                _labelList.push_back(std::move(textLabel));
            }
        }
    }

    std::shared_ptr<TileLayer> TileLayerBuilder::build(int layerIdx, boost::optional<CompOp> compOp, FloatFunction opacityFunc) {
        std::vector<std::shared_ptr<TileBitmap>> bitmapList;
        std::swap(bitmapList, _bitmapList);

        appendGeometry();
        std::vector<std::shared_ptr<TileGeometry>> geometryList;
        std::swap(geometryList, _geometryList);

        std::vector<std::shared_ptr<TileLabel>> labelList;
        std::swap(labelList, _labelList);
        std::for_each(labelList.begin(), labelList.end(), [layerIdx](const std::shared_ptr<TileLabel>& label) { label->setPriority(layerIdx); });

        return std::make_shared<TileLayer>(layerIdx, std::move(compOp), std::move(opacityFunc), std::move(bitmapList), std::move(geometryList), std::move(labelList));
    }

    float TileLayerBuilder::calculateScale(VertexArray<cglib::vec2<float>>& values) {
        float maxValue = 0.0f;
        for (const cglib::vec2<float>& value : values) {
            maxValue = std::max(maxValue, std::max(std::abs(value(0)), std::abs(value(1))));
        }
        float scale = 32768.0f;
        while (scale > 1.0f / 65536.0f) {
            if (maxValue * scale <= 32767.0f) {
                break;
            }
            scale *= 0.5f;
        }
        return scale;
    }

    boost::optional<cglib::mat3x3<float>> TileLayerBuilder::flipTransform(const boost::optional<cglib::mat3x3<float>>& transform) {
        if (!transform) {
            return transform;
        }
        return cglib::scale3_matrix(cglib::vec3<float>(1, -1, 1)) * transform.get() * cglib::scale3_matrix(cglib::vec3<float>(1, -1, 1));
    }

    void TileLayerBuilder::appendGeometry() {
        if (_builderParameters.type == TileGeometry::Type::NONE) {
            return;
        }

        if (_builderParameters.strokeMap) {
            bool strokeUsed = std::any_of(_builderParameters.lineStrokeIds.begin(), _builderParameters.lineStrokeIds.begin() + _styleParameters.parameterCount, [](StrokeMap::StrokeId strokeId) { return strokeId != 0; });
            if (strokeUsed) {
                _styleParameters.pattern = _builderParameters.strokeMap->getBitmapPattern();
            }
        }
        else if (_builderParameters.glyphMap) {
            _styleParameters.pattern = _builderParameters.glyphMap->getBitmapPattern();
        }

        if (_styleParameters.pattern) {
            // Normalize U, V coordinates according to actual pattern bitmap dimensions
            float uScale = 1.0f / _styleParameters.pattern->bitmap->width;
            float vScale = 1.0f / _styleParameters.pattern->bitmap->height;
            for (std::size_t i = 0; i < _texCoords.size(); i++) {
                _texCoords[i](0) *= uScale;
                _texCoords[i](1) *= vScale;
            }
        }
        else {
            _texCoords.clear();
        }

        if (_styleParameters.transform && _builderParameters.type != TileGeometry::Type::POINT) {
            cglib::mat3x3<float> invTransTransform = cglib::transpose(cglib::inverse(_styleParameters.transform.get()));
            for (std::size_t i = 0; i < _binormals.size(); i++) {
                _binormals[i] = cglib::unit(cglib::transform_vector(_binormals[i], invTransTransform)) * cglib::length(_binormals[i]);
            }
        }
        appendGeometry(calculateScale(_vertices), calculateScale(_binormals), calculateScale(_texCoords), _vertices, _texCoords, _binormals, _heights, _attribs, _indices, _ids, 0, _vertices.size());

        _builderParameters = BuilderParameters();
        _styleParameters = TileGeometry::StyleParameters();
        _vertices.clear();
        _texCoords.clear();
        _binormals.clear();
        _heights.clear();
        _attribs.clear();
        _indices.clear();
        _ids.clear();
    }

    void TileLayerBuilder::appendGeometry(float verticesScale, float binormalsScale, float texCoordsScale, const VertexArray<cglib::vec2<float>>& vertices, const VertexArray<cglib::vec2<float>>& texCoords, const VertexArray<cglib::vec2<float>>& binormals, const VertexArray<float>& heights, const VertexArray<cglib::vec4<char>>& attribs, const VertexArray<unsigned int>& indices, const VertexArray<long long>& ids, std::size_t offset, std::size_t count) {
        if (count < 65536) {
            // Build geometry layout info
            TileGeometry::GeometryLayoutParameters geometryLayoutParameters;
            geometryLayoutParameters.vertexOffset = geometryLayoutParameters.vertexSize;
            geometryLayoutParameters.vertexSize += 2 * sizeof(short);

            geometryLayoutParameters.attribsOffset = geometryLayoutParameters.vertexSize;
            geometryLayoutParameters.vertexSize += 4 * sizeof(char);

            if (!texCoords.empty()) {
                geometryLayoutParameters.texCoordOffset = geometryLayoutParameters.vertexSize;
                geometryLayoutParameters.vertexSize += 2 * sizeof(short);
            }

            if (!binormals.empty()) {
                geometryLayoutParameters.binormalOffset = geometryLayoutParameters.vertexSize;
                geometryLayoutParameters.vertexSize += 2 * sizeof(short);
            }

            if (!heights.empty()) {
                geometryLayoutParameters.heightOffset = geometryLayoutParameters.vertexSize;
                geometryLayoutParameters.vertexSize += sizeof(float);
            }

            geometryLayoutParameters.vertexScale = verticesScale;
            geometryLayoutParameters.binormalScale = binormalsScale;
            geometryLayoutParameters.texCoordScale = texCoordsScale;

            // Interleave, compress actual geometry data
            VertexArray<unsigned char> compressedVertexGeometry;
            compressedVertexGeometry.fill(0, count * geometryLayoutParameters.vertexSize);
            for (std::size_t i = 0; i < count; i++) {
                unsigned char* baseCompressedPtr = &compressedVertexGeometry[i * geometryLayoutParameters.vertexSize];

                const cglib::vec2<float>& vertex = vertices[i + offset];
                short* compressedVertexPtr = reinterpret_cast<short*>(baseCompressedPtr + geometryLayoutParameters.vertexOffset);
                compressedVertexPtr[0] = static_cast<short>(vertex(0) * verticesScale);
                compressedVertexPtr[1] = static_cast<short>(vertex(1) * verticesScale);

                const cglib::vec4<char>& attrib = attribs[i + offset];
                char* compressedAttribsPtr = reinterpret_cast<char*>(baseCompressedPtr + geometryLayoutParameters.attribsOffset);
                compressedAttribsPtr[0] = attrib(0);
                compressedAttribsPtr[1] = attrib(1);
                compressedAttribsPtr[2] = attrib(2);
                compressedAttribsPtr[3] = attrib(3);

                if (!texCoords.empty()) {
                    const cglib::vec2<float>& texCoord = texCoords[i + offset];
                    short* compressedTexCoordPtr = reinterpret_cast<short*>(baseCompressedPtr + geometryLayoutParameters.texCoordOffset);
                    compressedTexCoordPtr[0] = static_cast<short>(texCoord(0) * texCoordsScale);
                    compressedTexCoordPtr[1] = static_cast<short>(texCoord(1) * texCoordsScale);
                }

                if (!binormals.empty()) {
                    const cglib::vec2<float>& binormal = binormals[i + offset];
                    short* compressedBinormalPtr = reinterpret_cast<short*>(baseCompressedPtr + geometryLayoutParameters.binormalOffset);
                    compressedBinormalPtr[0] = static_cast<short>(binormal(0) * binormalsScale);
                    compressedBinormalPtr[1] = static_cast<short>(binormal(1) * binormalsScale);
                }

                if (!heights.empty()) {
                    float height = heights[i + offset];
                    float* compressedHeightPtr = reinterpret_cast<float*>(baseCompressedPtr + geometryLayoutParameters.heightOffset);
                    compressedHeightPtr[0] = height;
                }
            }
                
            // Compress indices
            VertexArray<unsigned short> compressedIndices;
            compressedIndices.reserve(indices.size());
            for (std::size_t i = 0; i < indices.size(); i++) {
                unsigned int index = indices[i];
                compressedIndices.append(static_cast<unsigned short>(index - offset));
            }

            // Compress ids
            std::vector<std::pair<unsigned int, long long>> compressedIds;
            if (!ids.empty()) {
                std::size_t offset = 0;
                for (std::size_t i = 1; i < ids.size(); i++) {
                    if (ids[i] != ids[offset]) {
                        compressedIds.emplace_back(static_cast<unsigned int>(i - offset), ids[offset]);
                        offset = i;
                    }
                }
                compressedIds.emplace_back(static_cast<unsigned int>(ids.size() - offset), ids[offset]);
                compressedIds.shrink_to_fit();
            }

            auto geometry = std::make_shared<TileGeometry>(_builderParameters.type, _tileSize, _geomScale, _styleParameters, geometryLayoutParameters, std::move(compressedVertexGeometry), std::move(compressedIndices), std::move(compressedIds));
            _geometryList.push_back(std::move(geometry));
            return;
        }

        // Split indices into 2 sets, find minimum/maximum values for each set
        unsigned int maxIndex[2] = { 0, 0 };
        unsigned int minIndex[2] = { std::numeric_limits<unsigned int>::max(), std::numeric_limits<unsigned int>::max() };
        std::size_t splitPos = (indices.size() / 6) * 3;
        for (std::size_t i = 0; i < indices.size(); i++) {
            std::size_t n = (i < splitPos ? 0 : 1);
            minIndex[n] = std::min(minIndex[n], indices[i]);
            maxIndex[n] = std::max(maxIndex[n], indices[i]);
        }
            
        // If splitting provided no progress, quit (in practice, impossible)
        if (std::max(maxIndex[0] - minIndex[0], maxIndex[1] - minIndex[1]) + 1 == count) {
            return;
        }

        // Do recursive splitting/appending of each subset
        VertexArray<unsigned int> indices1;
        indices1.copy(indices, 0, splitPos);
        VertexArray<long long> ids1;
        ids1.copy(ids, 0, splitPos);
        appendGeometry(verticesScale, binormalsScale, texCoordsScale, vertices, texCoords, binormals, heights, attribs, indices1, ids1, minIndex[0], maxIndex[0] - minIndex[0] + 1);

        VertexArray<unsigned int> indices2;
        indices2.copy(indices, splitPos, indices.size() - splitPos);
        VertexArray<long long> ids2;
        ids2.copy(ids, splitPos, indices.size() - splitPos);
        appendGeometry(verticesScale, binormalsScale, texCoordsScale, vertices, texCoords, binormals, heights, attribs, indices2, ids2, minIndex[1], maxIndex[1] - minIndex[1] + 1);
    }

    bool TileLayerBuilder::tesselateGlyph(const cglib::vec2<float>& vertex, char styleIndex, const cglib::vec2<float>& pen, const cglib::vec2<float>& size, const GlyphMap::Glyph* glyph) {
        float u0 = 0, v0 = 0, u1 = 0, v1 = 0;
        cglib::vec2<float> p0 = pen, p3 = pen + size;
        cglib::vec4<char> attrib(styleIndex, 0, 0, 0);
        if (glyph) {
            u0 = static_cast<float>(glyph->x); // NOTE: u,v coordinates will be normalized when the layer is built
            v0 = static_cast<float>(glyph->y);
            u1 = static_cast<float>(glyph->x + glyph->width);
            v1 = static_cast<float>(glyph->y + glyph->height);
            attrib(1) = (glyph->sdfMode ? -1 : 1);
        }

        if (_clipBox.inside(vertex)) {
            unsigned int i0 = static_cast<unsigned int>(_vertices.size());
            _indices.append(i0 + 0, i0 + 1, i0 + 2);
            _indices.append(i0 + 0, i0 + 2, i0 + 3);

            _vertices.append(vertex, vertex, vertex, vertex);
            _texCoords.append(cglib::vec2<float>(u0, v1), cglib::vec2<float>(u1, v1), cglib::vec2<float>(u1, v0), cglib::vec2<float>(u0, v0));
            _binormals.append(p0, cglib::vec2<float>(p3(0), p0(1)), p3, cglib::vec2<float>(p0(0), p3(1)));
            _attribs.append(attrib, attrib, attrib, attrib);
        }

        return true;
    }

    bool TileLayerBuilder::tesselatePolygon(const std::vector<std::vector<cglib::vec2<float>>>& verticesList, char styleIndex, const PolygonStyle& style) {
        if (!_tessPoolAllocator) {
            _tessPoolAllocator = std::unique_ptr<PoolAllocator>(new PoolAllocator);
        }

        TESSalloc ma;
        memset(&ma, 0, sizeof(ma));
        ma.memalloc = [](void* userData, unsigned int size) { return reinterpret_cast<PoolAllocator*>(userData)->allocate(size); };
        ma.memfree = [](void* userData, void* ptr) {};
        ma.userData = _tessPoolAllocator.get();
        ma.extraVertices = 256; // realloc not provided, allow 256 extra vertices.

        TESStesselator* tess = tessNewTess(&ma);
        if (!tess) {
            return false;
        }
        for (const std::vector<cglib::vec2<float>>& points : verticesList) {
            TESSreal* coords = reinterpret_cast<TESSreal*>(_tessPoolAllocator->allocate(points.size() * 2 * sizeof(TESSreal)));
            for (std::size_t i = 0; i < points.size(); i++) {
                coords[i * 2 + 0] = static_cast<TESSreal>(points[i](0));
                coords[i * 2 + 1] = static_cast<TESSreal>(points[i](1));
            }
            tessAddContour(tess, 2, coords, 2 * sizeof(TESSreal), static_cast<int>(points.size()));
        }
        tessTesselate(tess, TESS_WINDING_ODD, TESS_POLYGONS, 3, 2, 0);
        const TESSreal* coords = tessGetVertices(tess);
        const int* elements = tessGetElements(tess);
        const int vertexCount = tessGetVertexCount(tess);
        const int elementCount = tessGetElementCount(tess);

        float du_dx = 0.0f, dv_dy = 0.0f;
        if (style.pattern) {
            du_dx = _tileSize / style.pattern->widthScale;
            dv_dy = _tileSize / style.pattern->heightScale;
        }

        unsigned int offset = static_cast<unsigned int>(_vertices.size());
        for (int i = 0; i < vertexCount; i++) {
            cglib::vec2<float> p(static_cast<float>(coords[i * 2 + 0]), static_cast<float>(coords[i * 2 + 1]));
            cglib::vec2<float> uv(p(0) * du_dx + 0.5f, p(1) * dv_dy + 0.5f);

            _vertices.append(p);
            _texCoords.append(uv);
        }
        _attribs.fill(cglib::vec4<char>(styleIndex, 0, 0, 0), _vertices.size() - offset);

        cglib::bbox2<float> clipBox(cglib::vec2<float>(0.0f, 0.0f), cglib::vec2<float>(1.0f, 1.0f));
        for (int i = 0; i < elementCount * 3; i += 3) {
            int i0 = elements[i + 0];
            int i1 = elements[i + 1];
            int i2 = elements[i + 2];
            if (i0 == TESS_UNDEF || i1 == TESS_UNDEF || i2 == TESS_UNDEF) {
                continue;
            }
            i0 += offset;
            i1 += offset;
            i2 += offset;

            cglib::bbox2<float> bounds(_vertices[i0]);
            bounds.add(_vertices[i1]);
            bounds.add(_vertices[i2]);
            if (clipBox.inside(bounds)) {
                _indices.append(i0, i1, i2);
            }
        }

        tessDeleteTess(tess);
        _tessPoolAllocator->reset(); // reuse last allocated block from the start
        return true;
    }

    bool TileLayerBuilder::tesselatePolygon3D(const std::vector<std::vector<cglib::vec2<float>>>& verticesList, float minHeight, float maxHeight, char styleIndex, const Polygon3DStyle& style) {
        cglib::bbox2<float> clipBox(cglib::vec2<float>(0.0f, 0.0f), cglib::vec2<float>(1.0f, 1.0f));
        if (minHeight != maxHeight) {
            for (const std::vector<cglib::vec2<float>>& points : verticesList) {
                std::size_t j = points.size() - 1;
                for (std::size_t i = 0; i < points.size(); i++) {
                    cglib::bbox2<float> bounds(points[i]);
                    bounds.add(points[j]);
                    if (clipBox.inside(bounds)) {
                        cglib::vec2<float> tangent(cglib::unit(points[i] - points[j]));
                        cglib::vec2<float> binormal = cglib::vec2<float>(tangent(1), -tangent(0));

                        unsigned int i0 = static_cast<unsigned int>(_vertices.size());
                        _vertices.append(points[i], points[j], points[j]);
                        _binormals.append(binormal, binormal, binormal);
                        _heights.append(minHeight, minHeight, maxHeight);
                        _attribs.append(cglib::vec4<char>(styleIndex, 1, 0, 0), cglib::vec4<char>(styleIndex, 1, 0, 0), cglib::vec4<char>(styleIndex, 1, 1, 0));
                        _indices.append(i0 + 0, i0 + 1, i0 + 2);

                        unsigned int i1 = static_cast<unsigned int>(_vertices.size());
                        _vertices.append(points[j], points[i], points[i]);
                        _binormals.append(binormal, binormal, binormal);
                        _heights.append(maxHeight, maxHeight, minHeight);
                        _attribs.append(cglib::vec4<char>(styleIndex, 1, 1, 0), cglib::vec4<char>(styleIndex, 1, 1, 0), cglib::vec4<char>(styleIndex, 1, 0, 0));
                        _indices.append(i1 + 0, i1 + 1, i1 + 2);
                    }

                    j = i;
                }
            }
        }

        if (!_tessPoolAllocator) {
            _tessPoolAllocator = std::unique_ptr<PoolAllocator>(new PoolAllocator);
        }

        TESSalloc ma;
        memset(&ma, 0, sizeof(ma));
        ma.memalloc = [](void* userData, unsigned int size) { return reinterpret_cast<PoolAllocator*>(userData)->allocate(size); };
        ma.memfree = [](void* userData, void* ptr) {};
        ma.userData = _tessPoolAllocator.get();
        ma.extraVertices = 256; // realloc not provided, allow 256 extra vertices.

        TESStesselator* tess = tessNewTess(&ma);
        if (!tess) {
            return false;
        }
        for (const std::vector<cglib::vec2<float>>& points : verticesList) {
            TESSreal* coords = reinterpret_cast<TESSreal*>(_tessPoolAllocator->allocate(points.size() * 2 * sizeof(TESSreal)));
            for (std::size_t i = 0; i < points.size(); i++) {
                coords[i * 2 + 0] = static_cast<TESSreal>(points[i](0));
                coords[i * 2 + 1] = static_cast<TESSreal>(points[i](1));
            }
            tessAddContour(tess, 2, coords, 2 * sizeof(TESSreal), static_cast<int>(points.size()));
        }
        tessTesselate(tess, TESS_WINDING_ODD, TESS_POLYGONS, 3, 2, 0);
        const TESSreal* coords = tessGetVertices(tess);
        const int* elements = tessGetElements(tess);
        const int vertexCount = tessGetVertexCount(tess);
        const int elementCount = tessGetElementCount(tess);

        unsigned int offset = static_cast<unsigned int>(_vertices.size());
        for (int i = 0; i < vertexCount; i++) {
            cglib::vec2<float> p(static_cast<float>(coords[i * 2 + 0]), static_cast<float>(coords[i * 2 + 1]));

            _vertices.append(p);
        }
        _binormals.fill(cglib::vec2<float>(0, 0), _vertices.size() - offset);
        _heights.fill(maxHeight, _vertices.size() - offset);
        _attribs.fill(cglib::vec4<char>(styleIndex, 0, 1, 0), _vertices.size() - offset);

        for (int i = 0; i < elementCount * 3; i += 3) {
            int i0 = elements[i + 0];
            int i1 = elements[i + 1];
            int i2 = elements[i + 2];
            if (i0 == TESS_UNDEF || i1 == TESS_UNDEF || i2 == TESS_UNDEF) {
                continue;
            }
            i0 += offset;
            i1 += offset;
            i2 += offset;

            cglib::bbox2<float> bounds(_vertices[i0]);
            bounds.add(_vertices[i1]);
            bounds.add(_vertices[i2]);
            if (clipBox.inside(bounds)) {
                _indices.append(i2, i1, i0);
            }
        }

        tessDeleteTess(tess);
        _tessPoolAllocator->reset(); // reuse last allocated block from the start
        return true;
    }

    bool TileLayerBuilder::tesselateLine(const std::vector<cglib::vec2<float>>& points, char styleIndex, const StrokeMap::Stroke* stroke, const LineStyle& style) {
        if (points.size() < 2) {
            return false;
        }

        float v0 = 0, v1 = 0, du_dl = 0;
        if (stroke) {
            v0 = stroke->y0 + 0.5f;
            v1 = stroke->y1 - 0.5f;
            du_dl = _tileSize / stroke->scale;
        }

        bool cycle = points.front() == points.back();
        bool endpoints = !cycle && style.capMode != LineCapMode::NONE;

        std::size_t i = 1;
        for (; i < points.size(); i++) {
            if (points[i] != points[i - 1]) {
                break;
            }
        }
        if (i == points.size()) {
            return false;
        }

        cglib::vec2<float> knotBinormal(0, 0);
        if (cycle) {
            std::size_t j = points.size() - 1;
            for (; j > 0; j--) {
                if (points[j] != points[j - 1]) {
                    break;
                }
            }
            if (j == 0) {
                return false;
            }

            cglib::vec2<float> prevTangent(cglib::unit(points[j] - points[j - 1]));
            cglib::vec2<float> prevBinormal = cglib::vec2<float>(prevTangent(1), -prevTangent(0));
            cglib::vec2<float> tangent(cglib::unit(points[i] - points[i - 1]));
            cglib::vec2<float> binormal = cglib::vec2<float>(tangent(1), -tangent(0));

            float dot = cglib::dot_product(binormal, prevBinormal);
            if (dot < MIN_MITER_DOT) {
                cycle = endpoints = false;
            }
            else {
                knotBinormal = cglib::unit(binormal + prevBinormal) * (1 / std::sqrt((1 + dot) / 2));
            }
        }

        cglib::vec2<float> binormal(0, 0), tangent(0, 0);
        float linePos = 0;
        {
            const cglib::vec2<float>& p0 = points[i - 1];
            const cglib::vec2<float>& p1 = points[i];
            float u0 = 0;
            cglib::vec2<float> dp(p1 - p0);
            linePos += cglib::length(dp);

            tangent = cglib::unit(dp);
            binormal = cglib::vec2<float>(tangent(1), -tangent(0));

            if (endpoints) {
                unsigned int i0 = static_cast<unsigned int>(_vertices.size()) + 2; // refer to the point that will be added after end point
                tesselateLineEndPoint(p0, u0, v0, v1, i0, -tangent, binormal, styleIndex, style);
            }

            _vertices.append(p0, p0);
            _texCoords.append(cglib::vec2<float>(u0, v0), cglib::vec2<float>(u0, v1));
            _binormals.append(cycle ? -knotBinormal : -binormal, cycle ? knotBinormal : binormal);
            _attribs.append(cglib::vec4<char>(styleIndex, 0, 1, 1), cglib::vec4<char>(styleIndex, 0, -1, 1));
        }

        while (++i < points.size()) {
            const cglib::vec2<float>& p0 = points[i - 1];
            const cglib::vec2<float>& p1 = points[i];
            if (p0 == p1) {
                continue;
            }
            float u0 = linePos * du_dl;
            cglib::vec2<float> dp(p1 - p0);
            linePos += cglib::length(dp);

            cglib::vec2<float> prevBinormal = binormal;
            tangent = cglib::unit(dp);
            binormal = cglib::vec2<float>(tangent(1), -tangent(0));

            unsigned int i0 = static_cast<unsigned int>(_vertices.size());

            cglib::bbox2<float> bounds(p0);
            bounds.add(_vertices[i0 - 2]);
            if (_clipBox.inside(bounds)) {
                _indices.append(i0 - 2, i0 - 1, i0 + 0);
                _indices.append(i0 - 1, i0 + 0, i0 + 1);
            }

            float dot = cglib::dot_product(binormal, prevBinormal);
            if (dot < MIN_MITER_DOT) {
                _vertices.append(p0, p0);
                _texCoords.append(cglib::vec2<float>(u0, v0), cglib::vec2<float>(u0, v1));
                _binormals.append(-prevBinormal, prevBinormal);
                _attribs.append(cglib::vec4<char>(styleIndex, 0, 1, 1), cglib::vec4<char>(styleIndex, 0, -1, 1));

                _vertices.append(p0, p0);
                _texCoords.append(cglib::vec2<float>(u0, v0), cglib::vec2<float>(u0, v1));
                _binormals.append(-binormal, binormal);
                _attribs.append(cglib::vec4<char>(styleIndex, 0, 1, 1), cglib::vec4<char>(styleIndex, 0, -1, 1));
            }
            else {
                cglib::vec2<float> lerpedBinormal = cglib::unit(binormal + prevBinormal) * (1 / std::sqrt((1 + dot) / 2));

                _vertices.append(p0, p0);
                _texCoords.append(cglib::vec2<float>(u0, v0), cglib::vec2<float>(u0, v1));
                _binormals.append(-lerpedBinormal, lerpedBinormal);
                _attribs.append(cglib::vec4<char>(styleIndex, 0, 1, 1), cglib::vec4<char>(styleIndex, 0, -1, 1));
            }
        }
            
        {
            const cglib::vec2<float>& p0 = points[i - 1];
            float u0 = linePos * du_dl;

            unsigned int i0 = static_cast<unsigned int>(_vertices.size());
            
            cglib::bbox2<float> bounds(p0);
            bounds.add(_vertices[i0 - 2]);
            if (_clipBox.inside(bounds)) {
                _indices.append(i0 - 2, i0 - 1, i0 + 0);
                _indices.append(i0 - 1, i0 + 0, i0 + 1);
            }

            _vertices.append(p0, p0);
            _texCoords.append(cglib::vec2<float>(u0, v0), cglib::vec2<float>(u0, v1));
            _binormals.append(cycle ? -knotBinormal : -binormal, cycle ? knotBinormal : binormal);
            _attribs.append(cglib::vec4<char>(styleIndex, 0, 1, 1), cglib::vec4<char>(styleIndex, 0, -1, 1));

            if (endpoints) {
                tesselateLineEndPoint(p0, u0, v0, v1, i0, tangent, binormal, styleIndex, style);
            }
        }
        return true;
    }

    bool TileLayerBuilder::tesselateLineEndPoint(const cglib::vec2<float>& p0, float u0, float v0, float v1, unsigned int i0, const cglib::vec2<float>& tangent, const cglib::vec2<float>& binormal, char styleIndex, const LineStyle& style) {
        if (_clipBox.inside(p0)) {
            unsigned int i2 = static_cast<unsigned int>(_vertices.size());
            
            _vertices.append(p0, p0);
            _texCoords.append(cglib::vec2<float>(u0, v0), cglib::vec2<float>(u0, v1));
            _binormals.append(tangent - binormal, tangent + binormal);
            _attribs.append(cglib::vec4<char>(styleIndex, 1, 1, 1), cglib::vec4<char>(styleIndex, 1, -1, 1));

            _indices.append(i0 + 0, i0 + 1, i2 + 0);
            _indices.append(i0 + 1, i2 + 0, i2 + 1);
        }
        return true;
    }
} }
