#include "TileReader.h"
#include "ParserUtils.h"
#include "Symbolizer.h"
#include "Predicate.h"
#include "Expression.h"
#include "ExpressionContext.h"
#include "Rule.h"
#include "Filter.h"
#include "Map.h"

namespace carto { namespace mvt {
    TileReader::TileReader(std::shared_ptr<const Map> map, std::shared_ptr<const vt::TileTransformer> transformer, const SymbolizerContext& symbolizerContext) :
        _map(std::move(map)), _transformer(transformer), _symbolizerContext(symbolizerContext), _trueFilter(std::make_shared<Filter>(Filter::Type::FILTER, std::make_shared<ConstPredicate>(true)))
    {
    }

    std::shared_ptr<vt::Tile> TileReader::readTile(const vt::TileId& tileId) const {
        FeatureExpressionContext exprContext;
        exprContext.setTileId(tileId);
        exprContext.setAdjustedZoom(tileId.zoom + static_cast<int>(_symbolizerContext.getSettings().getZoomLevelBias()));
        exprContext.setNutiParameterValueMap(_symbolizerContext.getSettings().getNutiParameterValueMap());
        
        std::shared_ptr<const vt::BitmapPattern> backgroundPattern;
        if (!_map->getSettings().backgroundImage.empty()) {
            backgroundPattern = _symbolizerContext.getBitmapManager()->loadBitmapPattern(_map->getSettings().backgroundImage, 1.0f, 1.0f);
        }
        auto tileBackground = std::make_shared<vt::TileBackground>(_map->getSettings().backgroundColor, backgroundPattern);

        std::vector<std::shared_ptr<vt::TileLayer>> tileLayers;
        int layerIdx = 0;
        for (const std::shared_ptr<Layer>& layer : _map->getLayers()) {
            int styleIdx = 0;
            for (const std::string& styleName : layer->getStyleNames()) {
                const std::shared_ptr<Style>& style = _map->getStyle(styleName);
                if (!style) {
                    continue;
                }
                
                boost::optional<vt::CompOp> compOp;
                try {
                    if (!style->getCompOp().empty()) {
                        compOp = parseCompOp(style->getCompOp());
                    }
                }
                catch (const ParserException&) {
                    // ignore the error
                }

                vt::FloatFunction opacityFunc(style->getOpacity());

                int internalIdx = layerIdx * 65536 + static_cast<int>(layer->getStyleNames().size()) * 256 + styleIdx;
                vt::TileLayerBuilder tileLayerBuilder(tileId, internalIdx, _transformer->createTileVertexTransformer(tileId), _symbolizerContext.getSettings().getTileSize(), _symbolizerContext.getSettings().getGeometryScale());
                processLayer(layer, style, exprContext, tileLayerBuilder);

                std::shared_ptr<vt::TileLayer> tileLayer = tileLayerBuilder.buildTileLayer(compOp, opacityFunc);
                if (!(tileLayer->getBitmaps().empty() && tileLayer->getLabels().empty() && tileLayer->getGeometries().empty() && !compOp)) {
                    tileLayers.push_back(tileLayer);
                }
                styleIdx++;
            }
            layerIdx++;
        }
        return std::make_shared<vt::Tile>(tileId, _symbolizerContext.getSettings().getTileSize(), tileBackground, tileLayers);
    }

    void TileReader::processLayer(const std::shared_ptr<const Layer>& layer, const std::shared_ptr<const Style>& style, FeatureExpressionContext& exprContext, vt::TileLayerBuilder& layerBuilder) const {
        std::unordered_set<std::shared_ptr<const Expression>> styleFieldExprs = style->getReferencedFields(exprContext.getAdjustedZoom());
        std::unordered_set<std::string> styleFields;
        std::for_each(styleFieldExprs.begin(), styleFieldExprs.end(), [&](const std::shared_ptr<const Expression>& expr) {
            styleFields.insert(ValueConverter<std::string>::convert(expr->evaluate(exprContext)));
        });

        std::shared_ptr<Symbolizer> currentSymbolizer;
        FeatureCollection currentFeatureCollection;
        std::unordered_map<std::shared_ptr<const FeatureData>, std::vector<std::shared_ptr<Symbolizer>>> featureDataSymbolizersMap;

        if (auto featureIt = createFeatureIterator(layer)) {
            for (; featureIt->valid(); featureIt->advance()) {
                std::shared_ptr<const FeatureData> featureData = featureIt->getFeatureData();

                // Cache symbolizer evaluation for each feature data object
                auto symbolizersIt = featureDataSymbolizersMap.find(featureData);
                if (symbolizersIt == featureDataSymbolizersMap.end()) {
                    exprContext.setFeatureData(featureData);
                    std::vector<std::shared_ptr<Symbolizer>> symbolizers = findFeatureSymbolizers(style, exprContext);
                    symbolizersIt = featureDataSymbolizersMap.emplace(featureData, std::move(symbolizers)).first;
                }

                // Process symbolizers, try to batch as many calls together as possible
                for (const std::shared_ptr<Symbolizer>& symbolizer : symbolizersIt->second) {
                    if (std::shared_ptr<const Geometry> geometry = featureIt->getGeometry()) {
                        bool batch = false;
                        if (currentSymbolizer == symbolizer) {
                            batch = true;
                            if (!symbolizer->getParameterExpressions().empty()) {
                                for (const std::string& field : styleFields) {
                                    Value val1, val2;
                                    if (featureData->getVariable(field, val1) == currentFeatureCollection.getFeatureData(0)->getVariable(field, val2)) {
                                        if (val1 == val2) {
                                            continue;
                                        }
                                    }
                                    batch = false;
                                    break;
                                }
                            }
                        }

                        if (!batch) {
                            if (currentSymbolizer && currentFeatureCollection.size() > 0) {
                                exprContext.setFeatureData(currentFeatureCollection.getFeatureData(0));
                                currentSymbolizer->build(currentFeatureCollection, exprContext, _symbolizerContext, layerBuilder);
                            }
                            currentFeatureCollection.clear();
                            currentSymbolizer = symbolizer;
                        }

                        currentFeatureCollection.append(featureIt->getLocalId(), Feature(featureIt->getGlobalId(), geometry, featureData));
                    }
                }
            }

            // Flush the remaining batched features
            if (currentSymbolizer && currentFeatureCollection.size() > 0) {
                exprContext.setFeatureData(currentFeatureCollection.getFeatureData(0));
                currentSymbolizer->build(currentFeatureCollection, exprContext, _symbolizerContext, layerBuilder);
            }
        }
    }

    std::vector<std::shared_ptr<Symbolizer>> TileReader::findFeatureSymbolizers(const std::shared_ptr<const Style>& style, FeatureExpressionContext& exprContext) const {
        bool anyMatch = false;
        std::vector<std::shared_ptr<Symbolizer>> symbolizers;
        for (const std::shared_ptr<const Rule>& rule : style->getZoomRules(exprContext.getAdjustedZoom())) {
            std::shared_ptr<const Filter> filter = rule->getFilter();
            if (!filter) {
                filter = _trueFilter;
            }
            
            // Filter matching logic
            bool match = true;
            switch (filter->getType()) {
            case Filter::Type::FILTER:
                switch (style->getFilterMode()) {
                case Style::FilterMode::FIRST:
                    if (anyMatch) {
                        match = false;
                    }
                    else if (const std::shared_ptr<const Predicate>& pred = filter->getPredicate()) {
                        match = pred->evaluate(exprContext);
                    }
                    break;
                case Style::FilterMode::ALL:
                    if (const std::shared_ptr<const Predicate>& pred = filter->getPredicate()) {
                        match = pred->evaluate(exprContext);
                    }
                    break;
                }
                anyMatch = anyMatch || match;
                break;
            case Filter::Type::ELSEFILTER:
                match = !anyMatch;
                break;
            case Filter::Type::ALSOFILTER:
                match = anyMatch;
                break;
            }

            // If match, add all rule symbolizers to the symbolizer list
            if (match) {
                symbolizers.insert(symbolizers.end(), rule->getSymbolizers().begin(), rule->getSymbolizers().end());
            }
        }
        return symbolizers;
    }
} }
