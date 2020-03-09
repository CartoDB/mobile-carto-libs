#include "Address.h"
#include "AddressInterpolator.h"
#include "FeatureReader.h"

#include <sqlite3pp.h>

namespace carto { namespace geocoding {
    bool Address::loadFromDB(sqlite3pp::database& db, std::uint64_t encodedId, const std::string& language, const PointConverter& converter) {
        unsigned int entityId = static_cast<unsigned int>(encodedId & 0xffffffffU);
        unsigned int elementIndex = static_cast<unsigned int>(encodedId >> 32);
        
        sqlite3pp::query query(db, "SELECT type, features, housenumbers FROM entities WHERE id=:id");
        query.bind(":id", entityId);
        for (auto qit = query.begin(); qit != query.end(); qit++) {
            type = static_cast<EntityType>(qit->get<int>(0));

            // Feature reader
            EncodingStream featureStream(qit->get<const void*>(1), qit->get<const void*>(1) ? qit->column_bytes(1) : 0);
            EncodingStream houseNumberStream(qit->get<const void*>(2), qit->get<const void*>(2) ? qit->column_bytes(2) : 0);
            FeatureReader featureReader(featureStream, converter);
            AddressInterpolator interpolator(houseNumberStream);

            // Decode house number
            if (elementIndex) {
                std::pair<std::uint64_t, std::vector<Feature>> idFeatures = interpolator.readAddressesAndFeatures(featureReader).at(elementIndex - 1);
                sqlite3pp::query query1(db, "SELECT n.name FROM names n WHERE n.id=:id AND (n.lang IS NULL or n.lang=:lang) ORDER BY n.lang ASC");
                query1.bind(":id", idFeatures.first);
                query1.bind(":lang", language.c_str());
                for (auto qit1 = query1.begin(); qit1 != query1.end(); qit1++) {
                    houseNumber = qit1->get<const char*>(0);
                }
                features = idFeatures.second;
            }
            else {
                houseNumber.clear();
                std::vector<std::uint64_t> ids = interpolator.getAddresses();
                for (std::uint64_t id : ids) {
                    sqlite3pp::query query1(db, "SELECT n.name FROM names n WHERE n.id=:id AND (n.lang IS NULL or n.lang=:lang) ORDER BY n.lang ASC");
                    query1.bind(":id", id);
                    query1.bind(":lang", language.c_str());
                    for (auto qit1 = query1.begin(); qit1 != query1.end(); qit1++) {
                        if (!houseNumber.empty()) {
                            houseNumber.append(",");
                        }
                        houseNumber.append(qit1->get<const char*>(0));
                    }
                }
                features.clear();
                while (!featureStream.eof()) {
                    std::vector<Feature> featureCollection = featureReader.readFeatureCollection();
                    features.insert(features.end(), featureCollection.begin(), featureCollection.end());
                }
            }

            // Load names
            sqlite3pp::query query1(db, "SELECT n.name, n.type FROM entitynames en, names n WHERE en.entity_id=:id AND en.name_id=n.id AND (n.lang IS NULL or n.lang=:lang) ORDER BY n.lang ASC");
            query1.bind(":id", entityId);
            query1.bind(":lang", language.c_str());
            for (auto qit1 = query1.begin(); qit1 != query1.end(); qit1++) {
                std::string value = qit1->get<const char*>(0);
                switch (static_cast<FieldType>(qit1->get<int>(1))) {
                case FieldType::NONE:
                    break;
                case FieldType::COUNTRY:
                    country = value;
                    break;
                case FieldType::REGION:
                    region = value;
                    break;
                case FieldType::COUNTY:
                    county = value;
                    break;
                case FieldType::LOCALITY:
                    locality = value;
                    break;
                case FieldType::NEIGHBOURHOOD:
                    neighbourhood = value;
                    break;
                case FieldType::STREET:
                    street = value;
                    break;
                case FieldType::POSTCODE:
                    postcode = value;
                    break;
                case FieldType::NAME:
                    name = value;
                    break;
                case FieldType::HOUSENUMBER: // not really used
                    houseNumber = value;
                    break;
                }
            }

            // Load categories    
            categories.clear();
            sqlite3pp::query query2(db, "SELECT c.category FROM entitycategories ec, categories c WHERE ec.entity_id=:id AND ec.category_id=c.id");
            query2.bind(":id", entityId);
            for (auto qit2 = query2.begin(); qit2 != query2.end(); qit2++) {
                categories.insert(qit2->get<const char*>(0));
            }
            return true;
        }
        return false;
    }

    bool Address::merge(const Address& address) {
        if (address.type == type && address.country == country && address.region == region && address.county == county && address.locality == locality && address.neighbourhood == neighbourhood && address.street == street && address.name == name && address.houseNumber == houseNumber) {
            // Merge features and categories
            features.insert(features.end(), address.features.begin(), address.features.end());
            categories.insert(address.categories.begin(), address.categories.end());
            return true;
        }
        return false;
    }

    bool Address::equal(const Address& address, bool compareHouseNumbers) const {
        if (address.country == country && address.region == region && address.county == county && address.locality == locality && address.neighbourhood == neighbourhood && address.street == street && address.name == name) {
            return compareHouseNumbers ? address.houseNumber == houseNumber : true;
        }
        return false;
    }

    std::string Address::toString() const {
        std::string str;
        if (!name.empty()) {
            str += name;
        }
        if (!houseNumber.empty()) {
            str += (str.empty() ? "" : ", ") + houseNumber;
        }
        if (!street.empty()) {
            str += (str.empty() ? "" : (houseNumber.empty() ? ", " : " ")) + street;
        }
        if (!neighbourhood.empty()) {
            str += (str.empty() ? "" : ", ") + neighbourhood;
        }
        if (!locality.empty()) {
            str += (str.empty() ? "" : ", ") + locality;
        }
        if (!county.empty()) {
            str += (str.empty() ? "" : ", ") + county;
        }
        if (!region.empty()) {
            str += (str.empty() ? "" : ", ") + region;
        }
        if (!country.empty()) {
            str += (str.empty() ? "" : ", ") + country;
        }
        if (!postcode.empty()) {
            str += (str.empty() ? "" : ", ") + postcode;
        }
        return str;
    }
} }
