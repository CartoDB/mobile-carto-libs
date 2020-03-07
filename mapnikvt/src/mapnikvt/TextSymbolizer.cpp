#include "TextSymbolizer.h"
#include "ParserUtils.h"
#include "FontSet.h"
#include "Expression.h"
#include "ExpressionOperator.h"
#include "vt/FontManager.h"

#include <memory>
#include <vector>
#include <limits>

#include <boost/algorithm/string.hpp>

namespace carto { namespace mvt {
    void TextSymbolizer::build(const FeatureCollection& featureCollection, const FeatureExpressionContext& exprContext, const SymbolizerContext& symbolizerContext, vt::TileLayerBuilder& layerBuilder) {
        std::lock_guard<std::mutex> lock(_mutex);

        updateBindings(exprContext);

        if (_sizeFunc == vt::FloatFunction(0)) {
            return;
        }
        
        std::shared_ptr<vt::Font> font = getFont(symbolizerContext);
        if (!font) {
            _logger->write(Logger::Severity::ERROR, "Failed to load text font " + (!_faceName.empty() ? _faceName : _fontSetName));
            return;
        }

        vt::CompOp compOp = convertCompOp(_compOp);

        bool clip = _clipDefined ? _clip : _allowOverlap;

        vt::TextFormatter formatter(font, _sizeStatic, getFormatterOptions(symbolizerContext));

        float fontScale = symbolizerContext.getSettings().getFontScale();
        vt::LabelOrientation placement = convertTextPlacement(_placement);
        float minimumDistance = _minimumDistance * std::pow(2.0f, -exprContext.getAdjustedZoom());

        vt::ColorFunction fillFunc = _functionBuilder.createColorOpacityFunction(_fillFunc, _opacityFunc);
        vt::FloatFunction sizeFunc = _functionBuilder.createChainedFloatFunction("multiply" + boost::lexical_cast<std::string>(fontScale), [fontScale](float size) { return size * fontScale; }, _sizeFunc);
        vt::ColorFunction haloFillFunc = _functionBuilder.createColorOpacityFunction(_haloFillFunc, _haloOpacityFunc);
        vt::FloatFunction haloRadiusFunc = _functionBuilder.createChainedFloatFunction("multiply" + boost::lexical_cast<std::string>(fontScale), [fontScale](float size) { return size * fontScale; }, _haloRadiusFunc);

        std::vector<std::pair<long long, std::tuple<vt::TileLayerBuilder::Vertex, std::string>>> textInfos;
        std::vector<std::pair<long long, vt::TileLayerBuilder::TextLabelInfo>> labelInfos;

        auto addText = [&](long long localId, long long globalId, const std::string& text, const boost::optional<vt::TileLayerBuilder::Vertex>& vertex, const vt::TileLayerBuilder::Vertices& vertices) {
            long long groupId = (_allowOverlap ? -1 : (minimumDistance > 0 ? (std::hash<std::string>()(text) & 0x7fffffff) : 0));
            
            if (clip) {
                if (vertex) {
                    textInfos.emplace_back(localId, std::make_tuple(*vertex, text));
                }
                else if (!vertices.empty()) {
                    textInfos.emplace_back(localId, std::make_tuple(vertices.front(), text));
                }
            }
            else {
                labelInfos.emplace_back(localId, vt::TileLayerBuilder::TextLabelInfo(globalId * 3 + 1, groupId, text, vertex, vertices, minimumDistance));
            }
        };

        auto flushTexts = [&](const cglib::mat3x3<float>& transform) {
            if (clip) {
                vt::TextStyle style(compOp, fillFunc, sizeFunc, haloFillFunc, haloRadiusFunc, _orientationAngle, fontScale, cglib::vec2<float>(0, 0), std::shared_ptr<vt::BitmapImage>(), transform);

                std::size_t textInfoIndex = 0;
                layerBuilder.addTexts([&](long long& id, vt::TileLayerBuilder::Vertex& vertex, std::string& text) {
                    if (textInfoIndex >= textInfos.size()) {
                        return false;
                    }
                    id = textInfos[textInfoIndex].first;
                    vertex = std::get<0>(textInfos[textInfoIndex].second);
                    text = std::get<1>(textInfos[textInfoIndex].second);
                    textInfoIndex++;
                    return true;
                }, style, formatter);

                textInfos.clear();
            }
            else {
                vt::TextLabelStyle style(placement, fillFunc, sizeFunc, haloFillFunc, haloRadiusFunc, true, _orientationAngle, fontScale, cglib::vec2<float>(0, 0), std::shared_ptr<vt::BitmapImage>());

                std::size_t labelInfoIndex = 0;
                layerBuilder.addTextLabels([&](long long& id, vt::TileLayerBuilder::TextLabelInfo& labelInfo) {
                    if (labelInfoIndex >= labelInfos.size()) {
                        return false;
                    }
                    id = labelInfos[labelInfoIndex].first;
                    labelInfo = labelInfos[labelInfoIndex].second;
                    labelInfoIndex++;
                    return true;
                }, style, formatter);

                labelInfos.clear();
            }
        };

        buildFeatureCollection(featureCollection, exprContext, symbolizerContext, formatter, placement, -1, addText);

        flushTexts(cglib::mat3x3<float>::identity());
    }

    void TextSymbolizer::bindParameter(const std::string& name, const std::string& value) {
        if (name == "name") {
            _textExpression = std::make_shared<VariableExpression>(value);
        }
        else if (name == "feature-id") {
            bind(&_featureId, parseExpression(value), &TextSymbolizer::convertId);
            _featureIdDefined = true;
        }
        else if (name == "face-name") {
            bind(&_faceName, parseStringExpression(value));
        }
        else if (name == "fontset-name") {
            bind(&_fontSetName, parseStringExpression(value));
        }
        else if (name == "placement") {
            bind(&_placement, parseStringExpression(value));
        }
        else if (name == "size") {
            bind(&_sizeFunc, parseExpression(value));
            bind(&_sizeStatic, parseExpression(value));
        }
        else if (name == "spacing") {
            bind(&_spacing, parseExpression(value));
        }
        else if (name == "fill") {
            bind(&_fillFunc, parseStringExpression(value), &TextSymbolizer::convertColor);
        }
        else if (name == "opacity") {
            bind(&_opacityFunc, parseExpression(value));
        }
        else if (name == "halo-fill") {
            bind(&_haloFillFunc, parseStringExpression(value), &TextSymbolizer::convertColor);
        }
        else if (name == "halo-opacity") {
            bind(&_haloOpacityFunc, parseExpression(value));
        }
        else if (name == "halo-radius") {
            bind(&_haloRadiusFunc, parseExpression(value));
        }
        else if (name == "halo-rasterizer") {
            // just ignore this
        }
        else if (name == "allow-overlap") {
            bind(&_allowOverlap, parseExpression(value));
        }
        else if (name == "clip") {
            bind(&_clip, parseExpression(value));
            _clipDefined = true;
        }
        else if (name == "minimum-distance") {
            bind(&_minimumDistance, parseExpression(value));
        }
        else if (name == "text-transform") {
            bind(&_textTransform, parseStringExpression(value));
        }
        else if (name == "orientation") {
            bind(&_orientationAngle, parseExpression(value));
            _orientationDefined = true;
        }
        else if (name == "dx") {
            bind(&_dx, parseExpression(value));
        }
        else if (name == "dy") {
            bind(&_dy, parseExpression(value));
        }
        else if (name == "avoid-edges") {
            // can ignore this, we are not clipping texts at tile boundaries
        }
        else if (name == "wrap-width") {
            bind(&_wrapWidth, parseExpression(value));
        }
        else if (name == "wrap-before") {
            bind(&_wrapBefore, parseExpression(value));
        }
        else if (name == "character-spacing") {
            bind(&_characterSpacing, parseExpression(value));
        }
        else if (name == "line-spacing") {
            bind(&_lineSpacing, parseExpression(value));
        }
        else if (name == "horizontal-alignment") {
            bind(&_horizontalAlignment, parseStringExpression(value));
        }
        else if (name == "vertical-alignment") {
            bind(&_verticalAlignment, parseStringExpression(value));
        }
        else if (name == "comp-op") {
            bind(&_compOp, parseStringExpression(value));
        }
        else {
            Symbolizer::bindParameter(name, value);
        }
    }

    std::string TextSymbolizer::getTransformedText(const std::string& text) const {
        if (_textTransform.empty()) {
            return text;
        }
        else if (_textTransform == "uppercase") {
            return toUpper(text);
        }
        else if (_textTransform == "lowercase") {
            return toLower(text);
        }
        else if (_textTransform == "capitalize") {
            return capitalize(text);
        }
        return text;
    }

    std::shared_ptr<vt::Font> TextSymbolizer::getFont(const SymbolizerContext& symbolizerContext) const {
        std::shared_ptr<vt::Font> font = symbolizerContext.getSettings().getFallbackFont();
        if (!_faceName.empty()) {
            font = symbolizerContext.getFontManager()->getFont(_faceName, font);
        }
        else if (!_fontSetName.empty()) {
            for (const std::shared_ptr<FontSet>& fontSet : _fontSets) {
                if (fontSet->getName() == _fontSetName) {
                    const std::vector<std::string>& faceNames = fontSet->getFaceNames();
                    for (auto it = faceNames.rbegin(); it != faceNames.rend(); it++) {
                        const std::string& faceName = *it;
                        std::shared_ptr<vt::Font> mainFont = symbolizerContext.getFontManager()->getFont(faceName, font);
                        if (mainFont) {
                            font = mainFont;
                        }
                    }
                    break;
                }
            }
        }
        return font;
    }

    cglib::bbox2<float> TextSymbolizer::calculateTextSize(const std::shared_ptr<vt::Font>& font, const std::string& text, const vt::TextFormatter& formatter) const {
        std::vector<vt::Font::Glyph> glyphs = formatter.format(text, formatter.getFontSize());
        cglib::bbox2<float> bbox = cglib::bbox2<float>::smallest();
        cglib::vec2<float> pen = cglib::vec2<float>(0, 0);
        for (const vt::Font::Glyph& glyph : glyphs) {
            if (glyph.codePoint == vt::Font::CR_CODEPOINT) {
                pen = cglib::vec2<float>(0, 0);
            }
            else {
                bbox.add(pen + glyph.offset);
                bbox.add(pen + glyph.offset + glyph.size);
            }

            pen += glyph.advance;
        }
        return bbox;
    }

    vt::TextFormatter::Options TextSymbolizer::getFormatterOptions(const SymbolizerContext& symbolizerContext) const {
        float fontScale = symbolizerContext.getSettings().getFontScale();
        cglib::vec2<float> offset(_dx * fontScale, -_dy * fontScale);
        cglib::vec2<float> alignment(_dx < 0 ? 1.0f : (_dx > 0 ? -1.0f : 0.0f), _dy < 0 ? 1.0f : (_dy > 0 ? -1.0f : 0.0f));
        if (_horizontalAlignment == "left") {
            alignment(0) = -1.0f;
        }
        else if (_horizontalAlignment == "middle") {
            alignment(0) = 0.0f;
        }
        else if (_horizontalAlignment == "right") {
            alignment(0) = 1.0f;
        }
        if (_verticalAlignment == "top") {
            alignment(1) = -1.0f;
        }
        else if (_verticalAlignment == "middle") {
            alignment(1) = 0.0f;
        }
        else if (_verticalAlignment == "bottom") {
            alignment(1) = 1.0f;
        }
        return vt::TextFormatter::Options(alignment, offset, _wrapBefore, _wrapWidth * fontScale, _characterSpacing, _lineSpacing);
    }

    vt::LabelOrientation TextSymbolizer::convertTextPlacement(const std::string& orientation) const {
        vt::LabelOrientation placement = convertLabelPlacement(orientation);
        if (placement != vt::LabelOrientation::LINE) {
            if (_orientationDefined) { // if orientation is explictly defined, use POINT placement
                return vt::LabelOrientation::POINT;
            }
        }
        return placement;
    }

    void TextSymbolizer::buildFeatureCollection(const FeatureCollection& featureCollection, const FeatureExpressionContext& exprContext, const SymbolizerContext& symbolizerContext, const vt::TextFormatter& formatter, vt::LabelOrientation placement, float bitmapSize, const std::function<void(long long localId, long long globalId, const std::string& text, const boost::optional<vt::TileLayerBuilder::Vertex>& vertex, const vt::TileLayerBuilder::Vertices& vertices)>& addText) {
        FeatureExpressionContext textExprContext(exprContext);
        for (std::size_t index = 0; index < featureCollection.size(); index++) {
            textExprContext.setFeatureData(featureCollection.getFeatureData(index));
            std::string text = getTransformedText(ValueConverter<std::string>::convert(_textExpression->evaluate(textExprContext)));
            std::size_t hash = std::hash<std::string>()(text);
            float textSize = bitmapSize < 0 ? (placement == vt::LabelOrientation::LINE ? calculateTextSize(formatter.getFont(), text, formatter).size()(0) : 0) : bitmapSize;

            long long localId = featureCollection.getLocalId(index);
            long long globalId = _featureIdDefined ? _featureId : combineId(featureCollection.getGlobalId(index), hash);
            const std::shared_ptr<const Geometry>& geometry = featureCollection.getGeometry(index);

            auto addLineTexts = [&](const std::vector<cglib::vec2<float>>& vertices) {
                if (_spacing <= 0) {
                    addText(localId, globalId, text, boost::optional<vt::TileLayerBuilder::Vertex>(), vertices);
                    return;
                }

                float linePos = 0;
                for (std::size_t i = 1; i < vertices.size(); i++) {
                    const cglib::vec2<float>& v0 = vertices[i - 1];
                    const cglib::vec2<float>& v1 = vertices[i];

                    float lineLen = cglib::length(v1 - v0) * symbolizerContext.getSettings().getTileSize();
                    if (i == 1) {
                        linePos = std::min(lineLen, _spacing) * 0.5f;
                    }
                    while (linePos < lineLen) {
                        cglib::vec2<float> pos = v0 + (v1 - v0) * (linePos / lineLen);
                        if (std::min(pos(0), pos(1)) > 0.0f && std::max(pos(0), pos(1)) < 1.0f) {
                            addText(localId, generateId(), text, pos, vertices);
                        }

                        linePos += _spacing + textSize;
                    }

                    linePos -= lineLen;
                }
            };

            if (auto pointGeometry = std::dynamic_pointer_cast<const PointGeometry>(geometry)) {
                for (const auto& vertex : pointGeometry->getVertices()) {
                    addText(localId, globalId, text, vertex, vt::TileLayerBuilder::Vertices());
                }
            }
            else if (auto lineGeometry = std::dynamic_pointer_cast<const LineGeometry>(geometry)) {
                if (placement == vt::LabelOrientation::LINE) {
                    for (const auto& vertices : lineGeometry->getVerticesList()) {
                        addLineTexts(vertices);
                    }
                }
                else {
                    for (const auto& vertices : lineGeometry->getVerticesList()) {
                        addText(localId, globalId, text, boost::optional<vt::TileLayerBuilder::Vertex>(), vertices);
                    }
                }
            }
            else if (auto polygonGeometry = std::dynamic_pointer_cast<const PolygonGeometry>(geometry)) {
                if (placement == vt::LabelOrientation::LINE) {
                    for (const auto& vertices : polygonGeometry->getClosedOuterRings(true)) {
                        addLineTexts(vertices);
                    }
                }
                else {
                    for (const auto& vertex : polygonGeometry->getSurfacePoints()) {
                        addText(localId, globalId, text, vertex, vt::TileLayerBuilder::Vertices());
                    }
                }
            }
            else {
                _logger->write(Logger::Severity::WARNING, "Unsupported geometry for TextSymbolizer/ShieldSymbolizer");
            }
        }
    }
} }
