/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_GEOCODING_ADDRESS_H_
#define _CARTO_GEOCODING_ADDRESS_H_

#include "Feature.h"
#include "FeatureReader.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <set>

namespace sqlite3pp {
    class database;
}

namespace carto { namespace geocoding {
    struct Address final {
        enum class EntityType {
            NONE, COUNTRY, REGION, COUNTY, LOCALITY, NEIGHBOURHOOD, STREET, RESERVED1, POI, ADDRESS
        };
        
        enum class FieldType {
            NONE, COUNTRY, REGION, COUNTY, LOCALITY, NEIGHBOURHOOD, STREET, POSTCODE, NAME, HOUSENUMBER
        };

        EntityType type;
        std::string country;
        std::string region;
        std::string county;
        std::string locality;
        std::string neighbourhood;
        std::string street;
        std::string postcode;
        std::string houseNumber;
        std::string name;
        std::vector<Feature> features;
        std::set<std::string> categories;

        bool loadFromDB(sqlite3pp::database& db, std::uint64_t encodedId, const std::string& language, const PointConverter& converter);

        bool equal(const Address& address, bool compareHouseNumbers) const;

        bool merge(const Address& address);

        std::string toString() const;
    };
} }

#endif
