/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_SGRE_RULE_H_
#define _CARTO_SGRE_RULE_H_

#include "Base.h"
#include "Graph.h"

#include <string>
#include <optional>
#include <array>
#include <vector>
#include <map>

#include <picojson/picojson.h>

namespace carto::sgre {
    class Rule final {
    public:
        Rule() = default;

        const std::optional<std::vector<std::string>>& getProfiles() const { return _profiles; }
        const std::optional<std::vector<FeatureFilter>>& getFilters() const { return _filters; }
        const std::optional<Graph::LinkMode>& getLinkMode() const { return _linkMode; }
        const std::optional<Graph::SearchCriteria>& getSearchCriteria() const { return _searchCriteria; }

        void apply(Graph::Attributes& attribs, bool forward) const;

        static Rule parse(const picojson::value& ruleDef);

    private:
        static FloatParameter readFloatParameter(const picojson::value& ruleDef, const std::string& paramName);
        static std::array<FloatParameter, 2> readDirectionalFloatParameters(const picojson::value& ruleDef, const std::string& paramName);

        std::optional<std::vector<std::string>> _profiles;
        std::optional<std::vector<FeatureFilter>> _filters;
        std::optional<Graph::LinkMode> _linkMode;
        std::optional<Graph::SearchCriteria> _searchCriteria;

        std::array<FloatParameter, 2> _speed;
        std::array<FloatParameter, 2> _zSpeed;
        std::array<FloatParameter, 2> _turnSpeed;
        std::array<FloatParameter, 2> _delay;
    };

    class RuleList final {
    public:
        RuleList() = default;
        explicit RuleList(std::vector<Rule> rules) : _rules(std::move(rules)) { }

        const std::vector<Rule>& getRules() const { return _rules; }
        
        void filter(const std::string& profile);

        void apply(Graph::Attributes& attribs, bool forward) const;

        static RuleList parse(const picojson::value& ruleListDef);

    private:
        std::vector<Rule> _rules;
    };
}

#endif
