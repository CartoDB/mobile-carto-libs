/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_SYMBOLIZERGENERATOR_H_
#define _CARTO_MAPNIKVT_SYMBOLIZERGENERATOR_H_

#include "Properties.h"

#include <memory>
#include <string>

#include <pugixml.hpp>

namespace carto::mvt {
    class Logger;
    class Symbolizer;
    
    class SymbolizerGenerator {
    public:
        explicit SymbolizerGenerator(std::shared_ptr<Logger> logger) : _logger(std::move(logger)) { }
        virtual ~SymbolizerGenerator() = default;

        virtual void generateSymbolizer(const Symbolizer& symbolizer, pugi::xml_node& symbolizerNode) const;

    protected:
        std::string getSymbolizerProperty(const Symbolizer& symbolizer, const Property& prop) const;

        const std::shared_ptr<Logger> _logger;
    };
}

#endif
