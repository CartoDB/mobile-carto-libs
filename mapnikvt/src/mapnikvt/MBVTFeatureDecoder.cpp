#include "MBVTFeatureDecoder.h"
#include "Logger.h"

#include "mbvtpackage/MBVTPackage.pb.h"

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <atomic>
#include <list>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <limits>

#include <stdext/miniz.h>

namespace carto { namespace mvt {
    class MBVTFeatureDecoder::MBVTFeatureIterator : public carto::mvt::FeatureDecoder::FeatureIterator {
    public:
        explicit MBVTFeatureIterator(const std::shared_ptr<const vector_tile::Tile>& tile, int layerIndex, const std::unordered_set<std::string>* fields, const cglib::mat3x3<float>& transform, const cglib::bbox2<float>& clipBox, float buffer, bool globalIdOverride, long long tileIdOffset, const std::shared_ptr<MBVTFeatureDecoder::FeatureDataCache>& featureDataCache) :
            _tile(tile), _layer(&tile->layers(layerIndex)), _transform(transform), _clipBox(clipBox), _buffer(buffer), _globalIdOverride(globalIdOverride), _tileIdOffset(tileIdOffset), _featureDataCache(featureDataCache)
        {
            _layerIndexOffset = static_cast<long long>(layerIndex) << 32;

            for (int i = 0; i < _layer->keys_size(); i++) {
                if (_layer->keys(i) == "id" || _layer->keys(i) == "cartodb_id") {
                    _idKey = i;
                }
                if (fields) {
                    auto it = fields->find(_layer->keys(i));
                    if (it != fields->end()) {
                        _fieldKeys.push_back(i);
                    }
                }
                else {
                    _fieldKeys.push_back(i);
                }
            }
        }

        bool findByLocalId(long long localId) {
            if (localId >= _layerIndexOffset && localId < _layerIndexOffset + _layer->features_size()) {
                _index = static_cast<int>(localId - _layerIndexOffset);
                return true;
            }
            return false;
        }

        virtual bool valid() const override {
            return _index < _layer->features_size();
        }

        virtual void advance() override {
            _index++;
        }

        virtual long long getLocalId() const override {
            return _layerIndexOffset + _index;
        }

        virtual long long getGlobalId() const override {
            if (_globalIdOverride) {
                return _tileIdOffset + _layerIndexOffset + _index;
            }

            const vector_tile::Tile::Feature& feature = _layer->features(_index);
            if (feature.id() != 0) {
                return feature.id();
            }
            for (int i = 0; i + 1 < feature.tags_size(); i += 2) {
                if (feature.tags(i) == _idKey) {
                    int valueIdx = feature.tags(i + 1);
                    if (valueIdx >= 0 && valueIdx < _layer->values_size()) {
                        const vector_tile::Tile::Value& value = _layer->values(valueIdx);
                        if (value.has_int_value()) {
                            return static_cast<long long>(value.int_value());
                        }
                        else if (value.has_sint_value()) {
                            return static_cast<long long>(value.sint_value());
                        }
                        else if (value.has_uint_value()) {
                            return static_cast<long long>(value.uint_value());
                        }
                        return 0;
                    }
                }
            }
            return 0;
        }

        virtual std::shared_ptr<const FeatureData> getFeatureData() const override {
            const vector_tile::Tile::Feature& feature = _layer->features(_index);
            std::vector<int> tags(_fieldKeys.size() + 1, -1);
            tags.back() = static_cast<int>(feature.type());
            for (int i = 0; i + 1 < feature.tags_size(); i += 2) {
                auto it = std::find(_fieldKeys.begin(), _fieldKeys.end(), feature.tags(i));
                if (it != _fieldKeys.end()) {
                    tags[it - _fieldKeys.begin()] = feature.tags(i + 1);
                }
            }

            auto it = _featureDataCache->find(tags);
            if (it != _featureDataCache->end()) {
                return it->second;
            }

            FeatureData::GeometryType geomType = convertGeometryType(feature.type());
            std::vector<std::pair<std::string, Value>> dataMap;
            dataMap.reserve(tags.size());
            for (std::size_t i = 0; i < _fieldKeys.size(); i++) {
                if (tags[i] >= 0 && tags[i] < _layer->values_size()) {
                    dataMap.emplace_back(_layer->keys(_fieldKeys[i]), convertValue(_layer->values(tags[i])));
                }
            }

            auto featureData = std::make_shared<FeatureData>(geomType, std::move(dataMap));
            _featureDataCache->emplace(std::move(tags), featureData);
            return featureData;
        }

        virtual std::shared_ptr<const Geometry> getGeometry() const override {
            std::vector<std::vector<cglib::vec2<float>>> verticesList;
            decodeGeometry(_layer->features(_index), verticesList, 1.0f / _layer->extent());
            if (_buffer > 0 && _layer->features(_index).type() == vector_tile::Tile::LINESTRING) {
                bufferGeometry(verticesList, _buffer);
            }

            cglib::bbox2<float> bbox = cglib::bbox2<float>::smallest();
            for (std::vector<cglib::vec2<float>>& vertices : verticesList) {
                for (cglib::vec2<float>& p : vertices) {
                    p = cglib::transform_point(p, _transform);
                    bbox.add(p);
                }
            }
            if (!bbox.inside(_clipBox)) {
                return std::shared_ptr<Geometry>();
            }

            switch (_layer->features(_index).type()) {
            case vector_tile::Tile::POINT:
                if (!verticesList.empty()) {
                    return std::make_shared<PointGeometry>(std::move(verticesList.front()));
                }
                return std::shared_ptr<Geometry>();
            case vector_tile::Tile::LINESTRING:
                return std::make_shared<LineGeometry>(std::move(verticesList));
            case vector_tile::Tile::POLYGON: {
                PolygonGeometry::PolygonList polygons;
                if (_layer->has_version() && _layer->version() > 1) {
                    auto it = std::find_if(verticesList.begin(), verticesList.end(), isRingCCW); // find first outer ring
                    while (it != verticesList.end()) {
                        auto it0 = it++;
                        it = std::find_if(it, verticesList.end(), isRingCCW); // find next outer ring
                        polygons.emplace_back(it0, it);
                    }
                }
                else {
                    polygons.push_back(std::move(verticesList));
                }
                return std::make_shared<PolygonGeometry>(std::move(polygons));
            }
            default:
                return std::shared_ptr<Geometry>();
            }
        }

    private:
        static FeatureData::GeometryType convertGeometryType(vector_tile::Tile::GeomType geomType) {
            switch (geomType) {
            case vector_tile::Tile::POINT:
                return FeatureData::GeometryType::POINT_GEOMETRY;
            case vector_tile::Tile::LINESTRING:
                return FeatureData::GeometryType::LINE_GEOMETRY;
            case vector_tile::Tile::POLYGON:
                return FeatureData::GeometryType::POLYGON_GEOMETRY;
            default:
                return FeatureData::GeometryType::NULL_GEOMETRY;
            }
        }

        static Value convertValue(const vector_tile::Tile::Value& val) {
            if (val.has_bool_value()) {
                return Value(val.bool_value());
            }
            else if (val.has_int_value()) {
                return Value(static_cast<long long>(val.int_value()));
            }
            else if (val.has_sint_value()) {
                return Value(static_cast<long long>(val.sint_value()));
            }
            else if (val.has_uint_value()) {
                return Value(static_cast<long long>(val.uint_value()));
            }
            else if (val.has_float_value()) {
                return Value(static_cast<double>(val.float_value()));
            }
            else if (val.has_double_value()) {
                return Value(val.double_value());
            }
            else if (val.has_string_value()) {
                return Value(val.string_value());
            }
            return Value();
        }

        static void decodeGeometry(const vector_tile::Tile::Feature& feature, std::vector<std::vector<cglib::vec2<float>>>& verticesList, float scale) {
            int cx = 0, cy = 0;
            int cmd = 0, length = 0;
            std::vector<cglib::vec2<float>> vertices;
            vertices.reserve(feature.geometry_size());
            for (int i = 0; i < feature.geometry_size(); ) {
                if (length == 0) {
                    int cmdLength = feature.geometry(i++);
                    length = cmdLength >> 3;
                    cmd = cmdLength & 7;
                    if (length == 0) {
                        continue;
                    }
                }

                length--;
                if ((cmd == 1 || cmd == 2) && i + 2 <= feature.geometry_size()) {
                    if (cmd == 1) {
                        if (!vertices.empty()) {
                            verticesList.emplace_back();
                            std::swap(verticesList.back(), vertices);
                        }
                    }
                    int dx = feature.geometry(i++);
                    int dy = feature.geometry(i++);
                    dx = ((dx >> 1) ^ (-(dx & 1)));
                    dy = ((dy >> 1) ^ (-(dy & 1)));
                    cx += dx;
                    cy += dy;
                    vertices.emplace_back(static_cast<float>(cx) * scale, static_cast<float>(cy) * scale);
                }
                else if (cmd == 7) {
                    if (!vertices.empty()) {
                        if (vertices.front() != vertices.back()) {
                            cglib::vec2<float> p = vertices.front();
                            vertices.emplace_back(p);
                        }
                    }
                }
            }
            if (!vertices.empty()) {
                verticesList.emplace_back();
                std::swap(verticesList.back(), vertices);
            }
        }

        static void bufferGeometry(std::vector<std::vector<cglib::vec2<float>>>& verticesList, float buffer) {
            for (std::vector<cglib::vec2<float>>& vertices : verticesList) {
                if (vertices.size() < 2) {
                    continue;
                }

                {
                    cglib::vec2<float>& p0 = vertices[0];
                    const cglib::vec2<float>& p1 = vertices[1];
                    if (p0(0) <= 0 || p0(0) >= 1 || p0(1) <= 0 || p0(1) >= 1) {
                        if (p1 != p0) {
                            p0 = p0 + cglib::unit(p0 - p1) * buffer;
                        }
                    }
                }

                {
                    const cglib::vec2<float>& p0 = vertices[vertices.size() - 2];
                    cglib::vec2<float>& p1 = vertices[vertices.size() - 1];
                    if (p1(0) <= 0 || p1(0) >= 1 || p1(1) <= 0 || p1(1) >= 1) {
                        if (p1 != p0) {
                            p1 = p1 + cglib::unit(p1 - p0) * buffer;
                        }
                    }
                }
            }
        }

        static bool isRingCCW(const std::vector<cglib::vec2<float>>& vertices) {
            double area = 0;
            if (!vertices.empty()) {
                for (std::size_t i = 1; i < vertices.size(); i++) {
                    area += vertices[i - 1](0) * vertices[i](1) - vertices[i](0) * vertices[i - 1](1);
                }
                area += vertices.back()(0) * vertices.front()(1) - vertices.front()(0) * vertices.back()(1);
            }
            return area > 0;
        }

        int _index = 0;
        int _idKey = -1;
        long long _layerIndexOffset = 0;
        std::vector<int> _fieldKeys;
        std::shared_ptr<const vector_tile::Tile> _tile;
        const vector_tile::Tile::Layer* _layer;
        const cglib::mat3x3<float> _transform;
        const cglib::bbox2<float> _clipBox;
        const float _buffer;
        const bool _globalIdOverride;
        const long long _tileIdOffset;
        mutable std::shared_ptr<MBVTFeatureDecoder::FeatureDataCache> _featureDataCache;

        static std::atomic<long long> _idCounter;
    };

    std::atomic<long long> MBVTFeatureDecoder::MBVTFeatureIterator::_idCounter = ATOMIC_VAR_INIT(1);

    MBVTFeatureDecoder::MBVTFeatureDecoder(const std::vector<unsigned char>& data, std::shared_ptr<Logger> logger) :
        _transform(cglib::mat3x3<float>::identity()), _clipBox(cglib::vec2<float>(-0.1f, -0.1f), cglib::vec2<float>(1.1f, 1.1f)), _buffer(0), _globalIdOverride(false), _tileIdOffset(0), _tile(), _layerMap(), _logger(std::move(logger))
    {
        std::vector<unsigned char> uncompressedData;
        if (miniz::inflate_gzip(data.data(), data.size(), uncompressedData)) {
            protobuf::message tileMsg(uncompressedData.data(), uncompressedData.size());
            _tile = std::make_shared<vector_tile::Tile>(tileMsg);
        }
        else {
            protobuf::message tileMsg(data.data(), data.size());
            _tile = std::make_shared<vector_tile::Tile>(tileMsg);
        }

        for (int i = 0; i < _tile->layers_size(); i++) {
            const std::string& name = _tile->layers(i).name();
            if (_layerMap.find(name) != _layerMap.end()) {
                _logger->write(Logger::Severity::ERROR, "Duplicate layer name: " + name);
            }
            else {
                _layerMap[name] = i;
            }
        }
    }

    void MBVTFeatureDecoder::setTransform(const cglib::mat3x3<float>& transform) {
        _transform = transform;
    }

    void MBVTFeatureDecoder::setClipBox(const cglib::bbox2<float>& clipBox) {
        _clipBox = clipBox;
    }

    void MBVTFeatureDecoder::setBuffer(float buffer) {
        _buffer = buffer;
    }

    void MBVTFeatureDecoder::setGlobalIdOverride(bool globalIdOverride, long long tileIdOffset) {
        _globalIdOverride = globalIdOverride;
        _tileIdOffset = tileIdOffset;
    }

    std::vector<std::string> MBVTFeatureDecoder::getLayerNames() const {
        std::vector<std::string> layerNames;
        for (int i = 0; i < _tile->layers_size(); i++) {
            layerNames.push_back(_tile->layers(i).name());
        }
        return layerNames;
    }

    std::shared_ptr<FeatureDecoder::FeatureIterator> MBVTFeatureDecoder::createLayerFeatureIterator(const std::string& name) const {
        auto layerIt = _layerMap.find(name);
        if (layerIt == _layerMap.end()) {
            return std::shared_ptr<FeatureIterator>();
        }
        if (_layerFeatureDataCache.find(name) == _layerFeatureDataCache.end()) { // flush the cache if previous layer was different
            _layerFeatureDataCache.clear();
        }
        std::shared_ptr<FeatureDataCache>& featureDataCache = _layerFeatureDataCache[name];
        if (!featureDataCache) {
            featureDataCache = std::make_shared<FeatureDataCache>();
        }
        return std::make_shared<MBVTFeatureIterator>(_tile, layerIt->second, nullptr, _transform, _clipBox, _buffer, _globalIdOverride, _tileIdOffset, featureDataCache);
    }

    bool MBVTFeatureDecoder::findFeature(long long localId, std::string& layerName, Feature& feature) const {
        for (int i = 0; i < _tile->layers_size(); i++) {
            auto featureDataCache = std::make_shared<FeatureDataCache>();
            MBVTFeatureIterator it(_tile, i, nullptr, _transform, _clipBox, _buffer, _globalIdOverride, _tileIdOffset, featureDataCache);
            if (it.findByLocalId(localId)) {
                layerName = _tile->layers(i).name();
                feature = Feature(it.getGlobalId(), it.getGeometry(), it.getFeatureData());
                return true;
            }
        }
        return false;
    }
} }
