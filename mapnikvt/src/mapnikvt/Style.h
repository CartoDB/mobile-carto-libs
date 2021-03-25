/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_STYLE_H_
#define _CARTO_MAPNIKVT_STYLE_H_

#include "Expression.h"
#include "Predicate.h"

#include <memory>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace carto { namespace mvt {
    class Rule;
    
    class Style final {
    public:
        enum class FilterMode {
            ALL,
            FIRST
        };

        explicit Style(std::string name, float opacity, std::string imageFilters, std::string compOp, FilterMode filterMode, std::vector<std::shared_ptr<const Rule>> rules);

        const std::string& getName() const { return _name; }
        float getOpacity() const { return _opacity; }
        const std::string& getImageFilters() const { return _imageFilters; }
        const std::string& getCompOp() const { return _compOp; }
        FilterMode getFilterMode() const { return _filterMode; }
        const std::vector<std::shared_ptr<const Rule>>& getRules() const { return _rules; }

        const std::vector<std::shared_ptr<const Rule>>& getZoomRules(int zoom) const;

        const std::vector<Expression>& getReferencedFields(int zoom) const;

        void optimizeRules();

    private:
        void rebuildZoomRuleMap();

        static std::optional<Predicate> buildOptimizedOrPredicate(const std::optional<Predicate>& pred1, const std::optional<Predicate>& pred2);

        const std::string _name;
        const float _opacity;
        const std::string _imageFilters;
        const std::string _compOp;
        const FilterMode _filterMode;
        std::vector<std::shared_ptr<const Rule>> _rules;
        std::unordered_map<int, std::vector<std::shared_ptr<const Rule>>> _zoomRuleMap;
        std::unordered_map<int, std::vector<Expression>> _zoomFieldExprsMap;
    };
} }

#endif
