/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_SYMBOLIZERCONTEXT_H_
#define _CARTO_MAPNIKVT_SYMBOLIZERCONTEXT_H_

#include "ExpressionContext.h"
#include "vt/BitmapManager.h"
#include "vt/FontManager.h"
#include "vt/StrokeMap.h"
#include "vt/GlyphMap.h"

#include <memory>

namespace carto::mvt {
    class SymbolizerContext final {
    public:
        struct Settings {
            explicit Settings(float tileSize, std::map<std::string, Value> nutiParameterValueMap, std::shared_ptr<const vt::Font> fallbackFont);

            float getTileSize() const { return _tileSize; }
            float getGeometryScale() const { return _geometryScale; }
            float getFontScale() const { return _fontScale; }
            float getZoomLevelBias() const { return _zoomLevelBias; }

            const std::map<std::string, Value>& getNutiParameterValueMap() const { return _nutiParameterValueMap; }
            const std::shared_ptr<const vt::Font>& getFallbackFont() const { return _fallbackFont; }

        private:
            float _tileSize;
            float _geometryScale;
            float _fontScale;
            float _zoomLevelBias;
            
            std::map<std::string, Value> _nutiParameterValueMap;
            std::shared_ptr<const vt::Font> _fallbackFont;
        };

        explicit SymbolizerContext(std::shared_ptr<vt::BitmapManager> bitmapManager, std::shared_ptr<vt::FontManager> fontManager, std::shared_ptr<vt::StrokeMap> strokeMap, std::shared_ptr<vt::GlyphMap> glyphMap, const Settings& settings) : _bitmapManager(std::move(bitmapManager)), _fontManager(std::move(fontManager)), _strokeMap(std::move(strokeMap)), _glyphMap(std::move(glyphMap)), _settings(settings) { }

        std::shared_ptr<vt::BitmapManager> getBitmapManager() const { return _bitmapManager; }
        std::shared_ptr<vt::FontManager> getFontManager() const { return _fontManager; }
        std::shared_ptr<vt::StrokeMap> getStrokeMap() const { return _strokeMap; }
        std::shared_ptr<vt::GlyphMap> getGlyphMap() const { return _glyphMap; }
        const Settings& getSettings() const { return _settings; }

    private:
        const std::shared_ptr<vt::BitmapManager> _bitmapManager;
        const std::shared_ptr<vt::FontManager> _fontManager;
        const std::shared_ptr<vt::StrokeMap> _strokeMap;
        const std::shared_ptr<vt::GlyphMap> _glyphMap;
        const Settings _settings;
    };
}

#endif
