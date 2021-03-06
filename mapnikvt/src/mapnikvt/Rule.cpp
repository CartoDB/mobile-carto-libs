#include "Rule.h"
#include "Expression.h"
#include "Filter.h"
#include "Predicate.h"
#include "PredicateUtils.h"
#include "Rule.h"
#include "Symbolizer.h"

#include <algorithm>

namespace carto { namespace mvt {
    Rule::Rule(std::string name, int minZoom, int maxZoom, std::shared_ptr<const Filter> filter, std::vector<std::shared_ptr<const Symbolizer>> symbolizers) : _name(std::move(name)), _minZoom(minZoom), _maxZoom(maxZoom), _filter(std::move(filter)), _symbolizers(std::move(symbolizers)) {
        rebuildReferencedFields();
    }

    void Rule::rebuildReferencedFields() {
        struct FieldExtractor {
            explicit FieldExtractor(std::set<std::string>& fields) : _fields(fields) { }

            void operator() (const std::shared_ptr<VariableExpression>& varExpr) {
                if (auto val = std::get_if<Value>(&varExpr->getVariableExpression())) {
                    std::string name = ValueConverter<std::string>::convert(*val);
                    if (!(ExpressionContext::isViewStateVariable(name) || ExpressionContext::isNutiVariable(name) || ExpressionContext::isMapnikVariable(name) || ExpressionContext::isZoomVariable(name))) {
                        _fields.insert(name);
                    }
                }
                else {
                    _fields.insert(std::string()); // generic expression, insert special marker that any fields can be used
                }
            }

        private:
            std::set<std::string>& _fields;
        };
        
        _referencedFilterFields.clear();
        if (_filter) {
            if (_filter->getPredicate()) {
                std::visit(PredicateVariableVisitor(FieldExtractor(_referencedFilterFields)), *_filter->getPredicate());
            }
        }

        _referencedSymbolizerFields.clear();
        for (const std::shared_ptr<const Symbolizer>& symbolizer : _symbolizers) {
            for (const std::string& paramName : symbolizer->getParameterNames()) {
                if (auto param = symbolizer->getParameter(paramName)) {
                    std::visit(ExpressionVariableVisitor(FieldExtractor(_referencedSymbolizerFields)), param->getExpression());
                }
            }
        }
    }
} }
