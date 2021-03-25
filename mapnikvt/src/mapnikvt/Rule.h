/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_RULE_H_
#define _CARTO_MAPNIKVT_RULE_H_

#include <memory>
#include <string>
#include <vector>

#include "Expression.h"

namespace carto { namespace mvt {
    class Filter;
    class Symbolizer;
    
    class Rule final {
    public:
        explicit Rule(std::string name, int minZoom, int maxZoom, std::shared_ptr<const Filter> filter, std::vector<std::shared_ptr<Symbolizer>> symbolizers) : _name(std::move(name)), _minZoom(minZoom), _maxZoom(maxZoom), _filter(std::move(filter)), _symbolizers(std::move(symbolizers)) { }

        const std::string& getName() const { return _name; }
        int getMinZoom() const { return _minZoom; }
        int getMaxZoom() const { return _maxZoom; }
        const std::shared_ptr<const Filter>& getFilter() const { return _filter; }
        const std::vector<std::shared_ptr<Symbolizer>>& getSymbolizers() const { return _symbolizers; }

        void gatherReferencedFields(std::vector<Expression>& fields) const;

    private:
        const std::string _name;
        const int _minZoom;
        const int _maxZoom;
        const std::shared_ptr<const Filter> _filter;
        const std::vector<std::shared_ptr<Symbolizer>> _symbolizers;
    };
} }

#endif
