/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_LINESYMBOLIZER_H_
#define _CARTO_MAPNIKVT_LINESYMBOLIZER_H_

#include "GeometrySymbolizer.h"

namespace carto { namespace mvt {
    class LineSymbolizer : public GeometrySymbolizer {
    public:
        explicit LineSymbolizer(std::shared_ptr<Logger> logger) : GeometrySymbolizer(std::move(logger)) {
            _strokeWidthExpression = std::make_shared<ConstExpression>(Value(1.0f));
            bind(&_strokeFunc, std::make_shared<ConstExpression>(Value(std::string("#000000"))), &LineSymbolizer::convertColor);
            bind(&_strokeWidthFunc, _strokeWidthExpression);
            bind(&_strokeOpacityFunc, std::make_shared<ConstExpression>(Value(1.0f)));
        }

        virtual void build(const FeatureCollection& featureCollection, const FeatureExpressionContext& exprContext, const SymbolizerContext& symbolizerContext, vt::TileLayerBuilder& layerBuilder) override;

    protected:
        constexpr static int DASH_SUPERSAMPLING_FACTOR = 2;
        constexpr static float DASH_PATTERN_SCALE = 0.75f;

        virtual void bindParameter(const std::string& name, const std::string& value) override;

        vt::LineCapMode convertLineCapMode(const std::string& lineCap) const;
        vt::LineJoinMode convertLineJoinMode(const std::string& lineJoin) const;

        static std::shared_ptr<vt::BitmapPattern> createDashBitmapPattern(const std::vector<float>& strokeDashArray, int height, vt::LineCapMode lineCap);

        vt::ColorFunction _strokeFunc; // vt::Color(0xff000000)
        vt::FloatFunction _strokeWidthFunc; // 1.0f
        vt::FloatFunction _strokeOpacityFunc; // 1.0f
        std::string _strokeLinejoin = "miter";
        std::string _strokeLinecap = "butt";
        std::string _strokeDashArray;
        std::shared_ptr<Expression> _strokeWidthExpression;
    };
} }

#endif
