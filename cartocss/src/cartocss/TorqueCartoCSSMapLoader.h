/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_CARTOCSS_TORQUECARTOCSSMAPLOADER_H_
#define _CARTO_CARTOCSS_TORQUECARTOCSSMAPLOADER_H_

#include <string>
#include <vector>
#include <memory>

#include "CartoCSSMapLoader.h"
#include "mapnikvt/TorqueMap.h"
#include "mapnikvt/Logger.h"

namespace carto::css {
    class TorqueCartoCSSMapLoader : protected CartoCSSMapLoader {
    public:
        explicit TorqueCartoCSSMapLoader(std::shared_ptr<AssetLoader> assetLoader, std::shared_ptr<mvt::Logger> logger) : CartoCSSMapLoader(std::move(assetLoader), std::move(logger)) { }

        using CartoCSSMapLoader::isIgnoreLayerPredicates;
        using CartoCSSMapLoader::setIgnoreLayerPredicates;

        std::shared_ptr<mvt::TorqueMap> loadMap(const std::string& cartoCSS) const;

    protected:
        mvt::TorqueMap::TorqueSettings loadTorqueSettings(const std::map<std::string, Value>& mapProperties) const;
    };
}

#endif
