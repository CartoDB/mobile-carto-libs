/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_SYMBOLIZER_H_
#define _CARTO_MAPNIKVT_SYMBOLIZER_H_

#include "FeatureCollection.h"
#include "Expression.h"
#include "ExpressionContext.h"
#include "ExpressionBinder.h"
#include "FunctionBuilder.h"
#include "ParserUtils.h"
#include "SymbolizerContext.h"
#include "Logger.h"
#include "vt/Color.h"
#include "vt/TileLayerBuilder.h"

#include <memory>
#include <mutex>
#include <functional>

#include <cglib/mat.h>

namespace carto { namespace mvt {
    class Symbolizer {
    public:
        virtual ~Symbolizer() = default;

        void setParameter(const std::string& name, const std::string& value);
        const std::map<std::string, std::string>& getParameterMap() const;
        const std::vector<std::shared_ptr<const Expression>>& getParameterExpressions() const;

        virtual void build(const FeatureCollection& featureCollection, const FeatureExpressionContext& exprContext, const SymbolizerContext& symbolizerContext, vt::TileLayerBuilder& layerBuilder) = 0;

    protected:
        explicit Symbolizer(std::shared_ptr<Logger> logger) : _logger(std::move(logger)) { }

        vt::CompOp convertCompOp(const std::string& compOp) const;
        vt::LabelOrientation convertLabelPlacement(const std::string& orientation) const;

        vt::Color convertColor(const Value& val) const;
        cglib::mat3x3<float> convertTransform(const Value& val) const;
        boost::optional<cglib::mat3x3<float>> convertOptionalTransform(const Value& val) const;

        virtual void bindParameter(const std::string& name, const std::string& value);

        static long long generateId();

        static long long getTextId(long long id, std::size_t hash);
        static long long getShieldId(long long id, std::size_t hash);
        static long long getBitmapId(long long id, const std::string& file);

        template <typename V>
        void bind(V* field, const std::shared_ptr<const Expression>& expr) {
            bindParameter(field, expr, std::function<V(const Value&)>(ValueConverter<V>::convert));
            if (!std::dynamic_pointer_cast<const ConstExpression>(expr)) {
                _parameterExprs.push_back(expr);
            }
        }

        template <typename V>
        void bind(V* field, const std::shared_ptr<const Expression>& expr, V(*convertFunc)(const Value&)) {
            bindParameter(field, expr, std::function<V(const Value&)>(convertFunc));
            if (!std::dynamic_pointer_cast<const ConstExpression>(expr)) {
                _parameterExprs.push_back(expr);
            }
        }

        template <typename V>
        void bind(V* field, const std::shared_ptr<const Expression>& expr, V(Symbolizer::*memberconvertFunc)(const Value&) const) {
            bindParameter(field, expr, [this, memberconvertFunc](const Value& val) -> V {
                return (this->*memberconvertFunc)(val);
            });
            if (!std::dynamic_pointer_cast<const ConstExpression>(expr)) {
                _parameterExprs.push_back(expr);
            }
        }

        template <typename V>
        void bind(vt::UnaryFunction<V, vt::ViewState>* field, const std::shared_ptr<const Expression>& expr) {
            bindParameter(field, expr, std::function<V(const Value&)>(ValueConverter<V>::convert));
            if (!std::dynamic_pointer_cast<const ConstExpression>(expr)) {
                _parameterExprs.push_back(expr);
            }
        }

        template <typename V>
        void bind(vt::UnaryFunction<V, vt::ViewState>* field, const std::shared_ptr<const Expression>& expr, V(*convertFunc)(const Value&)) {
            bindParameter(field, expr, std::function<V(const Value&)>(convertFunc));
            if (!std::dynamic_pointer_cast<const ConstExpression>(expr)) {
                _parameterExprs.push_back(expr);
            }
        }

        template <typename V>
        void bind(vt::UnaryFunction<V, vt::ViewState>* field, const std::shared_ptr<const Expression>& expr, V(Symbolizer::*memberconvertFunc)(const Value&) const) {
            bindParameter(field, expr, [this, memberconvertFunc](const Value& val) -> V {
                return (this->*memberconvertFunc)(val);
            });
            if (!std::dynamic_pointer_cast<const ConstExpression>(expr)) {
                _parameterExprs.push_back(expr);
            }
        }

        void updateBindings(const FeatureExpressionContext& exprContext) {
            _boolBinder.update(exprContext);
            _intBinder.update(exprContext);
            _floatBinder.update(exprContext);
            _colorBinder.update(exprContext);
            _stringBinder.update(exprContext);
            _matrixBinder.update(exprContext);
            _optionalMatrixBinder.update(exprContext);
            _floatFunctionBinder.update(exprContext);
            _colorFunctionBinder.update(exprContext);
        }

        mutable std::mutex _mutex; // guards internal state as bindings may update it

        FunctionBuilder _functionBuilder;
        
        std::shared_ptr<Logger> _logger;

    private:
        void bindParameter(bool* field, const std::shared_ptr<const Expression>& expr, const std::function<bool(const Value&)>& convertFunc) {
            _boolBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(int* field, const std::shared_ptr<const Expression>& expr, const std::function<int(const Value&)>& convertFunc) {
            _intBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(float* field, const std::shared_ptr<const Expression>& expr, const std::function<float(const Value&)>& convertFunc) {
            _floatBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(vt::Color* field, const std::shared_ptr<const Expression>& expr, const std::function<vt::Color(const Value&)>& convertFunc) {
            _colorBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(std::string* field, const std::shared_ptr<const Expression>& expr, const std::function<std::string(const Value&)>& convertFunc) {
            _stringBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(cglib::mat<float, 3, cglib::float_traits<float>>* field, const std::shared_ptr<const Expression>& expr, const std::function<cglib::mat<float, 3, cglib::float_traits<float>>(const Value&)>& convertFunc) {
            _matrixBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(boost::optional<cglib::mat<float, 3, cglib::float_traits<float>>>* field, const std::shared_ptr<const Expression>& expr, const std::function<boost::optional<cglib::mat<float, 3, cglib::float_traits<float>>>(const Value&)>& convertFunc) {
            _optionalMatrixBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(vt::FloatFunction* field, const std::shared_ptr<const Expression>& expr, const std::function<float(const Value&)>& convertFunc) {
            _floatFunctionBinder.bind(field, expr, convertFunc);
        }

        void bindParameter(vt::ColorFunction* field, const std::shared_ptr<const Expression>& expr, const std::function<vt::Color(const Value&)>& convertFunc) {
            _colorFunctionBinder.bind(field, expr, convertFunc);
        }

        ExpressionBinder<bool> _boolBinder;
        ExpressionBinder<int> _intBinder;
        ExpressionBinder<float> _floatBinder;
        ExpressionBinder<vt::Color> _colorBinder;
        ExpressionBinder<std::string> _stringBinder;
        ExpressionBinder<cglib::mat<float, 3, cglib::float_traits<float>>> _matrixBinder;
        ExpressionBinder<boost::optional<cglib::mat<float, 3, cglib::float_traits<float>>>> _optionalMatrixBinder;
        ExpressionFunctionBinder<float> _floatFunctionBinder;
        ExpressionFunctionBinder<vt::Color> _colorFunctionBinder;

        std::map<std::string, std::string> _parameterMap;
        
        std::vector<std::shared_ptr<const Expression>> _parameterExprs;
    };
} }

#endif
