#include "MarkersSymbolizer.h"
#include "ParserUtils.h"
#include "vt/BitmapCanvas.h"

#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

namespace carto { namespace mvt {
    void MarkersSymbolizer::build(const FeatureCollection& featureCollection, const FeatureExpressionContext& exprContext, const SymbolizerContext& symbolizerContext, vt::TileLayerBuilder& layerBuilder) {
        std::lock_guard<std::mutex> lock(_mutex);

        updateBindings(exprContext);

        if ((_widthDefined && _widthFunc == vt::FloatFunction(0)) || (_heightDefined && _heightFunc == vt::FloatFunction(0)) || _fillOpacity == 0) {
            return;
        }

        vt::CompOp compOp = convertCompOp(_compOp);

        bool clip = _clipDefined ? _clip : _allowOverlap;

        float fontScale = symbolizerContext.getSettings().getFontScale();
        vt::LabelOrientation placement = convertLabelPlacement(_placement);
        vt::LabelOrientation orientation = placement;
        if (_transformExpression) { // if rotation transform is explicitly defined, use point orientation
            if (containsRotationTransform(_transformExpression->evaluate(exprContext))) {
                orientation = vt::LabelOrientation::POINT;
            }
        }
        if (placement == vt::LabelOrientation::LINE && _spacing > 0) {
            orientation = vt::LabelOrientation::POINT; // we will apply custom rotation, thus use point orientation
        }

        vt::FloatFunction sizeFunc;
        float bitmapScaleX = fontScale, bitmapScaleY = fontScale;
        std::shared_ptr<const vt::BitmapImage> bitmapImage;
        std::string file = _file;
        float fillOpacity = _fillOpacity;
        if (!file.empty()) {
            bitmapImage = symbolizerContext.getBitmapManager()->loadBitmapImage(file, false, IMAGE_UPSAMPLING_SCALE);
            if (!bitmapImage || !bitmapImage->bitmap) {
                _logger->write(Logger::Severity::ERROR, "Failed to load marker bitmap " + file);
                return;
            }
            if (bitmapImage->bitmap->width < 1 || bitmapImage->bitmap->height < 1) {
                return;
            }
            
            if (_widthDefined && _widthStatic > 0) {
                if (_heightDefined && _heightStatic > 0) {
                    bitmapScaleY *= _heightStatic / _widthStatic;
                }
                else {
                    bitmapScaleY *= static_cast<float>(bitmapImage->bitmap->height) / bitmapImage->bitmap->width;
                }
                sizeFunc = _widthFunc;
            }
            else if (_heightDefined && _heightStatic > 0) {
                bitmapScaleX *= static_cast<float>(bitmapImage->bitmap->width) / bitmapImage->bitmap->height;
                sizeFunc = _heightFunc;
            }
            else {
                bitmapScaleY *= static_cast<float>(bitmapImage->bitmap->height) / bitmapImage->bitmap->width;
                sizeFunc = _functionBuilder.createFloatFunction(bitmapImage->bitmap->width * bitmapImage->scale);
            }
        }
        else {
            vt::Color fill = vt::Color::fromColorOpacity(_fill, _fillOpacity);
            vt::Color stroke = vt::Color::fromColorOpacity(_stroke, _strokeOpacity);
            bool ellipse = _markerType == "ellipse" || (_markerType.empty() && placement != vt::LabelOrientation::LINE);
            float bitmapWidth = (ellipse ? DEFAULT_CIRCLE_SIZE : DEFAULT_ARROW_WIDTH), bitmapHeight = (ellipse ? DEFAULT_CIRCLE_SIZE : DEFAULT_ARROW_HEIGHT);
            if (_widthDefined) { // NOTE: special case, if accept all values
                bitmapHeight = (_heightDefined ? _heightStatic : _widthStatic * bitmapHeight / bitmapWidth);
                bitmapWidth = _widthStatic;
                bitmapScaleY *= bitmapHeight / bitmapWidth;
                sizeFunc = _widthFunc;
            }
            else if (_heightDefined) { // NOTE: special case, accept all values
                bitmapWidth = _heightStatic * bitmapWidth / bitmapHeight;
                bitmapHeight = _heightStatic;
                bitmapScaleX *= bitmapWidth / bitmapHeight;
                sizeFunc = _heightFunc;
            }
            else {
                bitmapScaleY *= bitmapHeight / bitmapWidth;
                sizeFunc = _functionBuilder.createFloatFunction(bitmapWidth);
            }
            bitmapScaleX *= (_strokeWidthStatic + bitmapWidth) / bitmapWidth;
            bitmapScaleY *= (_strokeWidthStatic + bitmapHeight) / bitmapHeight;
            bitmapWidth = std::min(bitmapWidth, static_cast<float>(MAX_BITMAP_SIZE));
            bitmapHeight = std::min(bitmapHeight, static_cast<float>(MAX_BITMAP_SIZE));
            
            if (ellipse) {
                file = "__default_marker_ellipse_" + boost::lexical_cast<std::string>(bitmapWidth) + "_" + boost::lexical_cast<std::string>(bitmapHeight) + "_" + boost::lexical_cast<std::string>(fill.value()) + "_" + boost::lexical_cast<std::string>(_strokeWidthStatic) + "_" + boost::lexical_cast<std::string>(stroke.value()) + ".bmp";
                bitmapImage = symbolizerContext.getBitmapManager()->getBitmapImage(file);
                if (!bitmapImage) {
                    bitmapImage = makeEllipseBitmap(bitmapWidth * SUPERSAMPLING_FACTOR, bitmapHeight * SUPERSAMPLING_FACTOR, fill, std::abs(_strokeWidthStatic) * SUPERSAMPLING_FACTOR, stroke);
                    symbolizerContext.getBitmapManager()->storeBitmapImage(file, bitmapImage);
                }
            }
            else {
                file = "__default_marker_arrow_" + boost::lexical_cast<std::string>(bitmapWidth) + "_" + boost::lexical_cast<std::string>(bitmapHeight) + "_" + boost::lexical_cast<std::string>(fill.value()) + "_" + boost::lexical_cast<std::string>(_strokeWidthStatic) + "_" + boost::lexical_cast<std::string>(stroke.value()) + ".bmp";
                bitmapImage = symbolizerContext.getBitmapManager()->getBitmapImage(file);
                if (!bitmapImage) {
                    bitmapImage = makeArrowBitmap(bitmapWidth * SUPERSAMPLING_FACTOR, bitmapHeight * SUPERSAMPLING_FACTOR, fill, std::abs(_strokeWidthStatic) * SUPERSAMPLING_FACTOR, stroke);
                    symbolizerContext.getBitmapManager()->storeBitmapImage(file, bitmapImage);
                }
            }

            fillOpacity = 1.0f;
        }

        float bitmapSize = static_cast<float>(std::max(_widthStatic * fontScale, _heightStatic * fontScale));
        long long groupId = (_allowOverlap ? -1 : 0);

        float widthScale = bitmapScaleX / bitmapImage->scale / bitmapImage->bitmap->width;
        float heightScale = bitmapScaleY / bitmapImage->scale / bitmapImage->bitmap->height;
        vt::FloatFunction normalizedSizeFunc = _functionBuilder.createChainedFloatFunction("multiply" + boost::lexical_cast<std::string>(widthScale), [widthScale](float size) { return size * widthScale; }, sizeFunc);
        vt::ColorFunction fillFunc = _functionBuilder.createColorFunction(vt::Color::fromColorOpacity(vt::Color(1, 1, 1, 1), fillOpacity));

        std::vector<std::pair<long long, vt::TileLayerBuilder::Vertex>> pointInfos;
        std::vector<std::pair<long long, vt::TileLayerBuilder::PointLabelInfo>> labelInfos;

        auto addPoint = [&](long long localId, long long globalId, const boost::variant<vt::TileLayerBuilder::Vertex, vt::TileLayerBuilder::Vertices>& position) {
            if (clip) {
                if (auto vertex = boost::get<vt::TileLayerBuilder::Vertex>(&position)) {
                    pointInfos.emplace_back(localId, *vertex);
                }
                else if (auto vertices = boost::get<vt::TileLayerBuilder::Vertices>(&position)) {
                    if (!vertices->empty()) {
                        pointInfos.emplace_back(localId, vertices->front());
                    }
                }
            }
            else {
                labelInfos.emplace_back(localId, vt::TileLayerBuilder::PointLabelInfo(globalId * 3 + 0, groupId, position, 0));
            }
        };

        auto flushPoints = [&](const cglib::mat3x3<float>& transform) {
            boost::optional<cglib::mat3x3<float>> optTransform;
            if (transform != cglib::mat3x3<float>::identity() || heightScale != widthScale) {
                optTransform = transform * cglib::scale3_matrix(cglib::vec3<float>(1.0f, heightScale / widthScale, 1.0f));
            }

            if (clip) {
                vt::PointStyle style(compOp, fillFunc, normalizedSizeFunc, bitmapImage, optTransform);

                std::size_t pointInfoIndex = 0;
                layerBuilder.addPoints([&](long long& id, vt::TileLayerBuilder::Vertex& vertex) {
                    if (pointInfoIndex >= pointInfos.size()) {
                        return false;
                    }
                    id = pointInfos[pointInfoIndex].first;
                    vertex = pointInfos[pointInfoIndex].second;
                    pointInfoIndex++;
                    return true;
                }, style, symbolizerContext.getGlyphMap());
                
                pointInfos.clear();
            }
            else {
                vt::PointLabelStyle style(orientation, fillFunc, normalizedSizeFunc, false, bitmapImage, optTransform);

                std::size_t labelInfoIndex = 0;
                layerBuilder.addPointLabels([&](long long& id, vt::TileLayerBuilder::PointLabelInfo& labelInfo) {
                    if (labelInfoIndex >= labelInfos.size()) {
                        return false;
                    }
                    id = labelInfos[labelInfoIndex].first;
                    labelInfo = std::move(labelInfos[labelInfoIndex].second);
                    labelInfoIndex++;
                    return true;
                }, style, symbolizerContext.getGlyphMap());
                
                labelInfos.clear();
            }
        };

        auto addLinePoints = [&](long long localId, long long globalId, const std::vector<cglib::vec2<float>>& vertices) {
            if (_spacing <= 0) {
                addPoint(localId, globalId, vertices);
                return;
            }

            flushPoints(_transform); // NOTE: we need to flush previous points at this point as we will recalculate transform, which is part of the style

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
                        addPoint(localId, generateId(), pos);

                        cglib::vec2<float> dir = cglib::unit(v1 - v0);
                        cglib::mat3x3<float> dirTransform = cglib::mat3x3<float>::identity();
                        dirTransform(0, 0) = dir(0);
                        dirTransform(0, 1) = -dir(1);
                        dirTransform(1, 0) = dir(1);
                        dirTransform(1, 1) = dir(0);
                        flushPoints(dirTransform * _transform); // NOTE: we should flush to be sure that the point will not get buffered
                    }

                    linePos += _spacing + bitmapSize;
                }

                linePos -= lineLen;
            }
        };

        std::size_t hash = std::hash<std::string>()(file);
        for (std::size_t index = 0; index < featureCollection.size(); index++) {
            long long localId = featureCollection.getLocalId(index);
            long long globalId = _featureIdDefined ? _featureId : combineId(featureCollection.getGlobalId(index), hash);
            const std::shared_ptr<const Geometry>& geometry = featureCollection.getGeometry(index);

            if (auto pointGeometry = std::dynamic_pointer_cast<const PointGeometry>(geometry)) {
                for (const auto& vertex : pointGeometry->getVertices()) {
                    addPoint(localId, globalId, vertex);
                }
            }
            else if (auto lineGeometry = std::dynamic_pointer_cast<const LineGeometry>(geometry)) {
                if (placement == vt::LabelOrientation::LINE) {
                    for (const auto& vertices : lineGeometry->getVerticesList()) {
                        addLinePoints(localId, globalId, vertices);
                    }
                }
                else {
                    for (const auto& vertex : lineGeometry->getMidPoints()) {
                        addPoint(localId, globalId, vertex);
                    }
                }
            }
            else if (auto polygonGeometry = std::dynamic_pointer_cast<const PolygonGeometry>(geometry)) {
                if (placement == vt::LabelOrientation::LINE) {
                    for (const auto& vertices : polygonGeometry->getClosedOuterRings(true)) {
                        addLinePoints(localId, globalId, vertices);
                    }                  
                }
                else {
                    for (const auto& vertex : polygonGeometry->getSurfacePoints()) {
                        addPoint(localId, globalId, vertex);
                    }
                }
            }
            else {
                _logger->write(Logger::Severity::WARNING, "Unsupported geometry for MarkersSymbolizer");
            }
        }

        flushPoints(_transform);
    }

    void MarkersSymbolizer::bindParameter(const std::string& name, const std::string& value) {
        if (name == "file") {
            bind(&_file, parseStringExpression(value));
        }
        else if (name == "placement") {
            bind(&_placement, parseStringExpression(value));
        }
        else if (name == "marker-type") {
            bind(&_markerType, parseStringExpression(value));
        }
        else if (name == "feature-id") {
            bind(&_featureId, parseExpression(value), &MarkersSymbolizer::convertId);
            _featureIdDefined = true;
        }
        else if (name == "fill") {
            bind(&_fill, parseStringExpression(value), &MarkersSymbolizer::convertColor);
        }
        else if (name == "fill-opacity") {
            bind(&_fillOpacity, parseExpression(value));
        }
        else if (name == "width") {
            bind(&_widthFunc, parseExpression(value));
            bind(&_widthStatic, parseExpression(value));
            _widthDefined = true;
        }
        else if (name == "height") {
            bind(&_heightFunc, parseExpression(value));
            bind(&_heightStatic, parseExpression(value));
            _heightDefined = true;
        }
        else if (name == "stroke") {
            bind(&_stroke, parseStringExpression(value), &MarkersSymbolizer::convertColor);
        }
        else if (name == "stroke-opacity") {
            bind(&_strokeOpacity, parseExpression(value));
        }
        else if (name == "stroke-width") {
            bind(&_strokeWidthFunc, parseExpression(value));
            bind(&_strokeWidthStatic, parseExpression(value));
        }
        else if (name == "spacing") {
            bind(&_spacing, parseExpression(value));
        }
        else if (name == "allow-overlap") {
            bind(&_allowOverlap, parseExpression(value));
        }
        else if (name == "clip") {
            bind(&_clip, parseExpression(value));
            _clipDefined = true;
        }
        else if (name == "ignore-placement") {
            bind(&_ignorePlacement, parseExpression(value));
        }
        else if (name == "transform") {
            _transformExpression = parseStringExpression(value);
            bind(&_transform, _transformExpression, &MarkersSymbolizer::convertTransform);
        }
        else if (name == "comp-op") {
            bind(&_compOp, parseStringExpression(value));
        }
        else if (name == "opacity") { // binds to 2 parameters
            bind(&_fillOpacity, parseExpression(value));
            bind(&_strokeOpacity, parseExpression(value));
        }
        else {
            Symbolizer::bindParameter(name, value);
        }
    }
    
    bool MarkersSymbolizer::containsRotationTransform(const Value& val) {
        try {
            std::vector<std::shared_ptr<Transform>> transforms = parseTransformList(boost::lexical_cast<std::string>(val));
            for (const std::shared_ptr<Transform>& transform : transforms) {
                if (std::dynamic_pointer_cast<RotateTransform>(transform)) {
                    return true;
                }
            }
            return false;
        }
        catch (const ParserException&) {
            return false;
        }
    }

    std::shared_ptr<vt::BitmapImage> MarkersSymbolizer::makeEllipseBitmap(float width, float height, const vt::Color& color, float strokeWidth, const vt::Color& strokeColor) {
        int canvasWidth = static_cast<int>(std::ceil(width + strokeWidth));
        int canvasHeight = static_cast<int>(std::ceil(height + strokeWidth));
        vt::BitmapCanvas canvas(canvasWidth, canvasHeight, false);
        float x0 = canvasWidth * 0.5f, y0 = canvasHeight * 0.5f;
        if (strokeWidth > 0) {
            canvas.setColor(strokeColor);
            canvas.drawEllipse(x0, y0, (width + strokeWidth) * 0.5f, (height + strokeWidth) * 0.5f);
        }
        canvas.setColor(color);
        canvas.drawEllipse(x0, y0, (width - strokeWidth) * 0.5f, (height - strokeWidth) * 0.5f);
        return canvas.buildBitmapImage();
    }

    std::shared_ptr<vt::BitmapImage> MarkersSymbolizer::makeArrowBitmap(float width, float height, const vt::Color& color, float strokeWidth, const vt::Color& strokeColor) {
        int canvasWidth = static_cast<int>(std::ceil(width + strokeWidth));
        int canvasHeight = static_cast<int>(std::ceil(height + strokeWidth));
        float x0 = 0, x1 = std::ceil(canvasWidth - canvasHeight * 0.5f), y1 = canvasHeight * 1.0f / 3.0f, y2 = canvasHeight * 2.0f / 3.0f;
        vt::BitmapCanvas canvas(canvasWidth, canvasHeight, false);
        if (strokeWidth > 0) {
            canvas.setColor(strokeColor);
            canvas.drawRectangle(x0, y1 - strokeWidth, x1 - strokeWidth, y2 + strokeWidth);
            canvas.drawTriangle(x1 - strokeWidth, 0, x1 - strokeWidth, canvasHeight, canvasWidth, canvasHeight * 0.5f);
        }
        canvas.setColor(color);
        canvas.drawRectangle(x0 + strokeWidth, y1 + strokeWidth * 0.5f, x1, y2 - strokeWidth * 0.5f);
        canvas.drawTriangle(x1, strokeWidth * 2 * std::sqrt(2.0f), x1, canvasHeight - strokeWidth * 2 * std::sqrt(2.0f), canvasWidth - strokeWidth * 2, canvasHeight * 0.5f);
        return canvas.buildBitmapImage();
    }
} }
