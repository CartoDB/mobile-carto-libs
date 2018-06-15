#include "CartoCSSCompiler.h"

#include <set>

namespace carto { namespace css {
    void CartoCSSCompiler::compileMap(const StyleSheet& styleSheet, std::map<std::string, Value>& mapProperties) const {
        // Build flat property lists
        std::map<std::string, std::shared_ptr<const Expression>> variableMap;
        PredicateContext context;
        context.expressionContext = _context;
        context.expressionContext.variableMap = &variableMap;
        std::list<FilteredPropertyList> propertyLists;
        buildPropertyLists(styleSheet, context, propertyLists);

        // Gather map properties from property lists
        for (const FilteredPropertyList& propertyList : propertyLists) {
            if (propertyList.attachment.empty()) {
                for (const FilteredProperty& prop : propertyList.properties) {
                    if (prop.filters.empty()) {
                        boost::variant<Value, std::shared_ptr<const Expression>> propResult = prop.property.expression->evaluate(context.expressionContext);
                        if (auto val = boost::get<Value>(&propResult)) {
                            mapProperties[prop.property.field] = *val;
                        }
                    }
                }
            }
        }
    }
    
    void CartoCSSCompiler::compileLayer(const std::string& layerName, const StyleSheet& styleSheet, std::list<LayerAttachment>& layerAttachments) const {
        // Build flat property lists
        std::map<std::string, std::shared_ptr<const Expression>> variableMap;
        PredicateContext context;
        context.layerName = layerName;
        context.expressionContext = _context;
        context.expressionContext.variableMap = &variableMap;
        std::list<FilteredPropertyList> propertyLists;
        buildPropertyLists(styleSheet, context, propertyLists);

        // Sort and evaluate property list
        for (FilteredPropertyList& propertyList : propertyLists) {
            // Sort the properties by decreasing specificity
            propertyList.properties.sort([&](const FilteredProperty& prop1, const FilteredProperty& prop2) {
                return prop1.property.specificity > prop2.property.specificity;
            });

            // Build property set list
            for (FilteredProperty& prop : propertyList.properties) {
                // Try to evaluate property expression, store the result as expression (even when constant)
                boost::variant<Value, std::shared_ptr<const Expression>> propResult = prop.property.expression->evaluate(context.expressionContext);
                if (auto val = boost::get<Value>(&propResult)) {
                    prop.property.expression = std::make_shared<ConstExpression>(*val);
                }
                else if (auto expr = boost::get<std::shared_ptr<const Expression>>(propResult)) {
                    prop.property.expression = expr;
                }
            }
        }

        // Check if we can reuse existing result
        if (propertyLists != _cachedLayerAttachments.first) {
            std::list<LayerAttachment> layerAttachmentList;
            for (const FilteredPropertyList& propertyList : propertyLists) {
                buildLayerAttachment(propertyList, layerAttachmentList);
            }
            _cachedLayerAttachments = { std::move(propertyLists), std::move(layerAttachmentList) };
        }

        // Store result from the cache
        layerAttachments.insert(layerAttachments.end(), _cachedLayerAttachments.second.begin(), _cachedLayerAttachments.second.end());
    }

    void CartoCSSCompiler::buildPropertyLists(const StyleSheet& styleSheet, PredicateContext& context, std::list<FilteredPropertyList>& propertyLists) const {
        for (const StyleSheet::Element& element : styleSheet.getElements()) {
            if (auto decl = boost::get<VariableDeclaration>(&element)) {
                if (context.expressionContext.variableMap->find(decl->getVariable()) == context.expressionContext.variableMap->end()) {
                    (*context.expressionContext.variableMap)[decl->getVariable()] = decl->getExpression();
                }
            }
            else if (auto ruleSet = boost::get<RuleSet>(&element)) {
                buildPropertyList(*ruleSet, context, "", std::vector<std::shared_ptr<const Predicate>>(), propertyLists);
            }
        }
    }
    
    void CartoCSSCompiler::buildPropertyList(const RuleSet& ruleSet, const PredicateContext& context, const std::string& attachment, const std::vector<std::shared_ptr<const Predicate>>& filters, std::list<FilteredPropertyList>& propertyLists) const {
        // List of selectors to use
        std::vector<Selector> selectors = ruleSet.getSelectors();
        if (selectors.empty() && !context.layerName.empty()) {
            selectors.push_back(Selector()); // special case, empty selector list means 'all'
        }
        
        // Process all selectors
        for (const Selector& selector : selectors) {
            // Build filters for given selector
            std::vector<std::shared_ptr<const Predicate>> selectorFilters(filters);
            for (const std::shared_ptr<const Predicate>& pred : selector.getPredicates()) {
                if (std::dynamic_pointer_cast<const LayerPredicate>(pred)) {
                    if (_ignoreLayerPredicates) {
                        continue;
                    }
                }
                selectorFilters.push_back(pred);
            }
            
            // Check if the filter list is 'reachable'. Also, remove always true conditions from the filter
            bool unreachable = false;
            std::string selectorAttachment = attachment;
            std::vector<std::shared_ptr<const Predicate>> optimizedSelectorFilters;
            for (auto it = selectorFilters.begin(); it != selectorFilters.end(); ) {
                boost::tribool pred = (*it)->evaluate(context);
                if (!pred) {
                    unreachable = true;
                    break;
                }
                if (auto attachmentPred = std::dynamic_pointer_cast<const AttachmentPredicate>(*it)) {
                    selectorAttachment += "::" + attachmentPred->getAttachment();
                    it = selectorFilters.erase(it);
                    continue;
                }
                if (boost::indeterminate(pred)) { // ignore always true filters
                    optimizedSelectorFilters.push_back(*it);
                }
                it++;
            }
            if (unreachable) {
                continue;
            }
            
            // Process block elements
            std::set<std::string> blockFields;
            for (const Block::Element& element : ruleSet.getBlock().getElements()) {
                if (auto decl = boost::get<PropertyDeclaration>(&element)) {
                    if (blockFields.find(decl->getField()) != blockFields.end()) {
                        continue;
                    }
                    blockFields.insert(decl->getField());
                    
                    // Find property set list for current attachment
                    auto propertyListsIt = std::find_if(propertyLists.begin(), propertyLists.end(), [&](const FilteredPropertyList& propertyList) {
                        return propertyList.attachment == selectorAttachment;
                    });
                    if (propertyListsIt == propertyLists.end()) {
                        FilteredPropertyList propertyList;
                        propertyList.attachment = selectorAttachment;
                        propertyListsIt = propertyLists.insert(propertyListsIt, propertyList);
                    }
                    std::list<FilteredProperty>& properties = (*propertyListsIt).properties;
                    
                    // Add property
                    FilteredProperty prop;
                    prop.property.field = decl->getField();
                    prop.property.expression = decl->getExpression();
                    prop.property.specificity = calculateRuleSpecificity(selectorFilters, decl->getOrder());
                    prop.filters = optimizedSelectorFilters;
                    properties.push_back(prop);
                }
                else if (auto subRuleSet = boost::get<RuleSet>(&element)) {
                    // Recurse with subrule
                    buildPropertyList(*subRuleSet, context, selectorAttachment, selectorFilters, propertyLists);
                }
            }
        }
    }

    void CartoCSSCompiler::buildLayerAttachment(const FilteredPropertyList& propertyList, std::list<LayerAttachment>& layerAttachments) const {
        std::list<PropertySet> propertySets;
        for (const FilteredProperty& prop : propertyList.properties) {
            for (auto propertySetIt = propertySets.begin(); propertySetIt != propertySets.end(); propertySetIt++) {
                // Check if this attribute is already set for given property set
                auto attributeIt = propertySetIt->properties.find(prop.property.field);
                if (attributeIt != propertySetIt->properties.end()) {
                    if (attributeIt->second.specificity >= prop.property.specificity) {
                        continue;
                    }
                    if (attributeIt->second.expression->equals(*prop.property.expression)) {
                        continue;
                    }
                }

                // Build new property set by setting the attribute and combining filters
                PropertySet propertySet(*propertySetIt);
                propertySet.properties[prop.property.field] = prop.property;
                propertySet.filters.insert(propertySet.filters.end(), prop.filters.begin(), prop.filters.end());
                if (!optimizePropertySetFilters(propertySet)) {
                    continue;
                }

                // If filters did not change, replace existing filter otherwise we must insert the new filter and keep old one
                if (isRedundantPropertySet(propertySets.begin(), propertySetIt, propertySet)) {
                    continue;
                }
                if (propertySet.filters == propertySetIt->filters) {
                    *propertySetIt = std::move(propertySet);
                }
                else {
                    propertySets.insert(propertySetIt, std::move(propertySet));
                }
            }

            // Add the property set
            PropertySet propertySet;
            propertySet.properties[prop.property.field] = prop.property;
            propertySet.filters = prop.filters;
            if (!optimizePropertySetFilters(propertySet)) {
                continue;
            }

            if (isRedundantPropertySet(propertySets.begin(), propertySets.end(), propertySet)) {
                continue;
            }
            propertySets.push_back(std::move(propertySet));
        }

        // Add layer attachment
        LayerAttachment layerAttachment;
        layerAttachment.attachment = propertyList.attachment;
        layerAttachment.order = std::numeric_limits<int>::max();
        for (const PropertySet& propertySet : propertySets) {
            for (const std::pair<std::string, Property>& namedProp : propertySet.properties) {
                layerAttachment.order = std::min(layerAttachment.order, std::get<3>(namedProp.second.specificity));
            }
        }
        layerAttachment.propertySets = std::move(propertySets);
        layerAttachments.push_back(std::move(layerAttachment));
    }

    bool CartoCSSCompiler::optimizePropertySetFilters(PropertySet& propertySet) {
        for (auto filterIt1 = propertySet.filters.begin(); filterIt1 != propertySet.filters.end(); filterIt1++) {
            for (auto filterIt2 = filterIt1; ++filterIt2 != propertySet.filters.end(); ) {
                // Check if it is possible to satisfy both filters. If not, can skip this rule
                if (*filterIt1 != *filterIt2 && !(*filterIt2)->intersects(*filterIt1)) {
                    return false;
                }
            }
            for (auto filterIt2 = filterIt1; ++filterIt2 != propertySet.filters.end(); ) {
                // Check if the filter is subsumed by any other filter
                if (*filterIt1 == *filterIt2 || (*filterIt2)->contains(*filterIt1)) {
                    filterIt2 = propertySet.filters.erase(filterIt2);
                    filterIt2--;
                }
            }
        }
        return true;
    }
    
    bool CartoCSSCompiler::isRedundantPropertySet(std::list<PropertySet>::iterator begin, std::list<PropertySet>::iterator end, const PropertySet& propertySet) {
        for (auto it = end; it != begin; ) {
            --it;

            if (std::all_of(it->filters.begin(), it->filters.end(), [&](const std::shared_ptr<const Predicate>& filter) {
                if (std::find(propertySet.filters.begin(), propertySet.filters.end(), filter) != propertySet.filters.end()) {
                    return true;
                }
                return std::any_of(propertySet.filters.begin(), propertySet.filters.end(), [&](const std::shared_ptr<const Predicate>& propFilter) {
                    return filter->contains(propFilter);
                });
            })) {
                return true;
            }
        }
        return false;
    }
    
    CartoCSSCompiler::RuleSpecificity CartoCSSCompiler::calculateRuleSpecificity(const std::vector<std::shared_ptr<const Predicate>>& predicates, int order) {
        int layers = 0;
        int classes = 0;
        int filters = 0;
        for (const std::shared_ptr<const Predicate>& pred : predicates) {
            if (std::dynamic_pointer_cast<const LayerPredicate>(pred)) {
                layers++;
            }
            else if (std::dynamic_pointer_cast<const ClassPredicate>(pred)) {
                classes++;
            }
            else if (std::dynamic_pointer_cast<const AttachmentPredicate>(pred)) {
                // ignore
            }
            else {
                filters++;
            }
        }
        return std::make_tuple(layers, classes, filters, order);
    }
} }
