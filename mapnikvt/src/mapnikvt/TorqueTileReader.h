/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_TORQUETILEREADER_H_
#define _CARTO_MAPNIKVT_TORQUETILEREADER_H_

#include "TileReader.h"
#include "TorqueMap.h"

namespace carto { namespace mvt {
    class TorqueFeatureDecoder;
    
    class TorqueTileReader : public TileReader {
    public:
        explicit TorqueTileReader(std::shared_ptr<TorqueMap> map, int frame, bool loop, std::shared_ptr<const vt::TileTransformer> transformer, const SymbolizerContext& symbolizerContext, const TorqueFeatureDecoder& featureDecoder) : TileReader(std::move(map), std::move(transformer), symbolizerContext), _frame(frame), _loop(loop), _featureDecoder(featureDecoder) { }

    protected:
        virtual std::shared_ptr<vt::TileBackground> createTileBackground(const vt::TileId& tileId) const override;
        
        virtual std::shared_ptr<FeatureDecoder::FeatureIterator> createFeatureIterator(const std::shared_ptr<const Layer>& layer, const std::set<std::string>* fields) const override;

    private:
        const int _frame;
        const bool _loop;
        const TorqueFeatureDecoder& _featureDecoder;
    };
} }

#endif
