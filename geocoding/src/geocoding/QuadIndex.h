/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_GEOCODING_QUADINDEX_H_
#define _CARTO_GEOCODING_QUADINDEX_H_

#include "Geometry.h"
#include "GeometryReader.h"
#include "ProjUtils.h"

#include <cstdint>
#include <cmath>
#include <tuple>
#include <vector>
#include <functional>

namespace carto::geocoding {
    class QuadIndex {
    public:
        using Result = std::pair<std::uint64_t, double>;
        using GeometryInfo = std::pair<std::uint64_t, std::shared_ptr<Geometry>>;
        using GeometryInfoFinder = std::function<std::vector<GeometryInfo>(const std::vector<std::uint64_t>&, const PointConverter&)>;

        explicit QuadIndex(const GeometryInfoFinder& geomInfoFinder) : _geometryInfoFinder(geomInfoFinder) { }

        std::vector<Result> findGeometries(double lng, double lat, float radius) const {
            cglib::vec2<double> mercatorPos = wgs84ToWebMercator({ lng, lat });
            cglib::vec2<double> mercatorMeters = webMercatorMeters({ lng, lat });

            std::vector<Result> results;
            for (int level = MAX_LEVEL; level >= 0; level--) {
                auto tile0 = calculatePointTile(mercatorPos(0) - radius / mercatorMeters(0), mercatorPos(1) - radius / mercatorMeters(1), level);
                auto tile1 = calculatePointTile(mercatorPos(0) + radius / mercatorMeters(0), mercatorPos(1) + radius / mercatorMeters(1), level);
                std::vector<std::uint64_t> quadIndices;
                for (int yt = std::get<2>(tile0); yt <= std::get<2>(tile1); yt++) {
                    for (int xt = std::get<1>(tile0); xt <= std::get<1>(tile1); xt++) {
                        quadIndices.push_back(calculateTileQuadIndex(level, xt, yt));
                    }
                }

                std::vector<GeometryInfo> geomInfos = _geometryInfoFinder(quadIndices, [](const cglib::vec2<double>& pos) {
                    return wgs84ToWebMercator(pos);
                });
                for (const GeometryInfo& geomInfo : geomInfos) {
                    // TODO: -180/180 wrapping
                    cglib::vec2<double> boundsPoint = geomInfo.second->getBounds().nearest_point(mercatorPos);
                    cglib::vec2<double> boundsDiff = boundsPoint - mercatorPos;
                    double boundsDist = cglib::length(cglib::vec2<double>(boundsDiff(0) * mercatorMeters(0), boundsDiff(1) * mercatorMeters(1)));
                    if (boundsDist <= radius) {
                        cglib::vec2<double> point = geomInfo.second->calculateNearestPoint(mercatorPos);
                        cglib::vec2<double> diff = point - mercatorPos;
                        double dist = cglib::length(cglib::vec2<double>(diff(0) * mercatorMeters(0), diff(1) * mercatorMeters(1)));
                        if (dist <= radius) {
                            results.emplace_back(geomInfo.first, dist);
                        }
                    }
                }
            }

            return results;
        }

    private:
        static std::tuple<int, int, int> calculatePointTile(double xm, double ym, int zoom) {
            double d = consts::EARTH_RADIUS * consts::PI;
            double s = 2 * d / (1 << zoom);
            return std::tuple<int, int, int> { zoom, static_cast<int>(std::floor((xm + d) / s)), static_cast<int>(std::floor((ym + d) / s)) };
        }

        static std::tuple<double, double, double, double> calculateTileBounds(int zoom, int xt, int yt) {
            double d = consts::EARTH_RADIUS * consts::PI;
            double s = 2 * d / (1 << zoom);
            return std::tuple<double, double, double, double> { xt * s - d, yt * s - d, xt * s - d + s, yt * s - d + s };
        }

        static std::uint64_t calculateTileQuadIndex(int zoom, int xt, int yt) {
            return zoom + (((static_cast<std::uint64_t>(yt) << zoom) + static_cast<std::uint64_t>(xt)) << LEVEL_BITS);
        }

        static constexpr int MAX_LEVEL = 18;
        static constexpr int LEVEL_BITS = 5;

        const GeometryInfoFinder _geometryInfoFinder;
    };
}

#endif
