#include "Geocoder.h"
#include "FeatureReader.h"
#include "ProjUtils.h"
#include "AddressInterpolator.h"

#include <functional>
#include <algorithm>
#include <numeric>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <sqlite3pp.h>

namespace {
    std::string escapeSQLValue(const std::string& val) {
        return boost::replace_all_copy(val, "'", "''");
    }
}

namespace carto { namespace geocoding {
    void Geocoder::prepare(sqlite3pp::database& db) {
    }
    
    bool Geocoder::import(const std::shared_ptr<sqlite3pp::database>& db) {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        auto database = std::make_shared<Database>();
        database->id = "db" + boost::lexical_cast<std::string>(_databases.size());
        database->db = db;
        database->origin = getOrigin(*db);
        database->bounds = getBounds(*db);
        database->rankScale = getRankScale(*db);
        database->translationTable = getTranslationTable(*db);
        
        _databases.push_back(std::move(database));
        return true;
    }
    
    std::string Geocoder::getLanguage() const {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        return _language;
    }

    void Geocoder::setLanguage(const std::string& language) {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        _language = language;
        _addressCache.clear();
        _entityCache.clear();
        _nameCache.clear();
        _nameRankCache.clear();
        _nameMatchCache.clear();
    }

    unsigned int Geocoder::getMaxResults() const {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        return _maxResults;
    }

    void Geocoder::setMaxResults(unsigned int maxResults) {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        _maxResults = maxResults;
    }

    bool Geocoder::getAutocomplete() const {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        return _autocomplete;
    }

    void Geocoder::setAutocomplete(bool autocomplete) {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
         _autocomplete = autocomplete;
    }

    bool Geocoder::isFilterEnabled(Address::EntityType type) const {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        return std::find(_enabledFilters.begin(), _enabledFilters.end(), type) != _enabledFilters.end();
    }

    void Geocoder::setFilterEnabled(Address::EntityType type, bool enabled) {
        std::lock_guard<std::recursive_mutex> lock(_mutex);
        auto it = std::find(_enabledFilters.begin(), _enabledFilters.end(), type);
        if (enabled && it == _enabledFilters.end()) {
            _enabledFilters.push_back(type);
        }
        else if (!enabled && it != _enabledFilters.end()) {
            _enabledFilters.erase(it);
        }
    }

    std::vector<std::pair<Address, float>> Geocoder::findAddresses(const std::string& queryString, const Options& options) const {
        std::lock_guard<std::recursive_mutex> lock(_mutex);

        std::string queryStringLC = unistring::to_utf8string(unistring::to_lower(unistring::to_unistring(queryString)));
        std::string safeQueryString = boost::replace_all_copy(boost::replace_all_copy(queryStringLC, "%", ""), "_", "");
        boost::trim(safeQueryString);

        // Prepare autocomplete query string by appending % sign
        bool autocomplete = _autocomplete && safeQueryString.size() >= MIN_AUTOCOMPLETE_SIZE;

        // Do matching in 2 phases (exact/inexact), if required
        std::vector<Result> results;
        for (int pass = 0; pass < 2; pass++) {
            for (const std::shared_ptr<Database>& database : _databases) {
                if (options.bounds) {
                    if (!options.bounds->inside(database->bounds)) {
                        continue;
                    }
                }

                Query query;
                query.database = database;
                if (autocomplete) {
                    query.tokenList = TokenList::build(safeQueryString + (boost::trim_right_copy(queryString) != queryString ? " " : "%"));
                }
                else {
                    query.tokenList = TokenList::build(safeQueryString);
                }
                matchTokens(query, pass, query.tokenList);
                
                std::set<std::vector<std::pair<std::uint32_t, std::string>>> assignments;
                matchQuery(query, options, assignments, results);
            }
            if (!results.empty()) {
                break;
            }
        }

        // Reorder databases, keep databases with best matches first in the list for subsequent queries
        for (auto it = results.rbegin(); it != results.rend(); it++) {
            auto dbit = std::find(_databases.begin(), _databases.end(), it->database);
            if (dbit != _databases.end()) {
                std::rotate(_databases.begin(), dbit, dbit + 1);
            }
        }

        // Create address data from the results by merging consecutive results, if possible
        std::vector<std::pair<Address, float>> addresses;
        for (const Result& result : results) {
            if (addresses.size() >= _maxResults) {
                break;
            }

            Address address;
            std::string addrKey = result.database->id + std::string(1, 0) + boost::lexical_cast<std::string>(result.encodedId);
            if (!_addressCache.read(addrKey, address)) {
                address.loadFromDB(*result.database->db, result.encodedId, _language, [&result](const cglib::vec2<double>& pos) {
                    return result.database->origin + pos;
                });

                _addressQueryCounter++;
                _addressCache.put(addrKey, address);
            }

            // If we have already the same address in the list, drop the new one. Note that we ignore house numbers unless the existing record has also house numbers.
            bool keep = true;
            for (std::size_t i = 0; i < addresses.size(); i++) {
                if (addresses[i].first.equal(address, addresses[i].first.type == Address::EntityType::ADDRESS)) {
                    keep = false;
                    break;
                }
            }
            if (keep) {
                addresses.emplace_back(address, result.totalRank());
            }
        }
        return addresses;
    }

    void Geocoder::matchTokens(Query& query, int pass, TokenList& tokenList) const {
        for (int i = 0; i < tokenList.size(); i++) {
            std::string tokenValue = tokenList.tokens(TokenList::Span(i, 1)).front();

            // Do token translation, actual tokens are normalized relative to real names using translation table
            unistring::unistring translatedToken = getTranslatedToken(unistring::to_unistring(tokenValue), query.database->translationTable);

            // Build token info list for the token
            if (!translatedToken.empty()) {
                std::string sql = "SELECT id, token, typemask, namecount, idf FROM tokens WHERE ";
                if (pass > 0 && translatedToken.size() >= 2) {
                    sql += "token LIKE '" + escapeSQLValue(unistring::to_utf8string(translatedToken.substr(0, 2))) + "%' ORDER BY ABS(LENGTH(token) - " + boost::lexical_cast<std::string>(translatedToken.size()) + ") ASC, idf ASC LIMIT " + boost::lexical_cast<std::string>(TOKEN_QUERY_LIMIT);
                }
                else if (!translatedToken.empty() && translatedToken.back() == '%') {
                    sql += "token LIKE '" + escapeSQLValue(unistring::to_utf8string(translatedToken)) + "' ORDER BY LENGTH(token) ASC, idf ASC LIMIT " + boost::lexical_cast<std::string>(TOKEN_QUERY_LIMIT);
                }
                else {
                    sql += "token='" + escapeSQLValue(unistring::to_utf8string(translatedToken)) + "'";
                }

                std::string tokenKey = query.database->id + std::string(1, 0) + sql;
                std::vector<Token> tokens;
                if (!_tokenCache.read(tokenKey, tokens)) {
                    sqlite3pp::query sqlQuery(*query.database->db, sql.c_str());

                    for (auto qit = sqlQuery.begin(); qit != sqlQuery.end(); qit++) {
                        Token token;
                        token.id = qit->get<std::uint64_t>(0);
                        token.token = qit->get<const char*>(1);
                        token.typeMask = qit->get<std::uint32_t>(2);
                        token.count = qit->get<std::uint64_t>(3);
                        token.idf = static_cast<float>(qit->get<double>(4));
                        tokens.push_back(std::move(token));
                    }
                    tokens.shrink_to_fit();

                    _tokenQueryCounter++;
                    _tokenCache.put(tokenKey, tokens);
                }
                    
                std::uint32_t validTypeMask = 0;
                float minIDF = std::numeric_limits<float>::infinity();
                for (auto it = tokens.begin(); it != tokens.end(); ) {
                    std::vector<std::pair<std::string, float>> tokenIDFs = { { it->token, it->idf } };
                    if (calculateNameRank(query, it->token, unistring::to_utf8string(translatedToken), tokenIDFs) >= MIN_MATCH_THRESHOLD) {
                        minIDF = std::min(minIDF, it->idf);
                        validTypeMask |= it->typeMask;
                        it++;
                    }
                    else {
                        it = tokens.erase(it);
                    }
                }
                tokenList.setTag(i, tokens);
                tokenList.setIDF(i, minIDF);
                tokenList.setValidTypeMask(i, validTypeMask);
            }
        }
    }

    void Geocoder::matchQuery(Query& query, const Options& options, std::set<std::vector<std::pair<std::uint32_t, std::string>>>& assignments, std::vector<Result>& results) const {
        if (query.tokenList.unmatchedInvalidTokens() > 0) { // TODO: make 0 part of context
            return;
        }
        if (!results.empty()) {
            if (results.front().unmatchedTokens < query.tokenList.unmatchedInvalidTokens()) {
                return;
            }
        }
        
        if (assignments.find(query.tokenList.assignment()) != assignments.end()) {
            return;
        }
        assignments.insert(query.tokenList.assignment());

        // Enumerate token list
        query.tokenList.enumerateSpans([&](std::uint32_t typeMask, const TokenList::Span& span) -> bool {
            if (query.tokenList.unmatchedInvalidTokens() > 0) { // TODO: make 0 part of context
                return true;
            }
            if (!results.empty()) {
                if (results.front().unmatchedTokens < query.tokenList.unmatchedInvalidTokens()) {
                    return true;
                }
            }

            std::vector<std::vector<Token>> tokensList = query.tokenList.tags(span);
            std::string matchName = boost::algorithm::join(query.tokenList.tokens(span), " ");
            std::shared_ptr<std::vector<NameRank>> nameRanks;
            matchNames(query, tokensList, matchName, nameRanks);
            
            std::size_t resultCount = results.size();
            if (!nameRanks->empty()) {
                std::uint32_t nameTypeMask = std::accumulate(nameRanks->begin(), nameRanks->end(), std::uint32_t(0), [](std::uint32_t mask, const NameRank& nameRank) { return mask | (1 << static_cast<int>(nameRank.name->type)); });
                Query subQuery = query;
                subQuery.tokenList.assignTypeMask(span, nameTypeMask);
                subQuery.filtersList.push_back(std::move(nameRanks));
                matchQuery(subQuery, options, assignments, results);
            }
            return results.size() == resultCount || results.front().unmatchedTokens > 0 || results.front().matchRank < MIN_RANK_SETTLE_THRESHOLD;
        });

        if (query.tokenList.unmatchedTokens() == 0) { // TODO: make 0 part of context
            matchEntities(query, options, results);
        }
    }

    void Geocoder::matchNames(const Query& query, const std::vector<std::vector<Token>>& tokensList, const std::string& matchName, std::shared_ptr<std::vector<NameRank>>& nameRanks) const {
        if (tokensList.empty()) {
            return;
        }
        
        std::string nameKey = query.database->id + std::string(1, 0) + matchName;
        for (const std::vector<Token>& tokens : tokensList) {
            nameKey += std::string(1, 0);
            for (const Token& token : tokens) {
                nameKey += boost::lexical_cast<std::string>(token.id) + ";";
            }
        }
        if (!_nameRankCache.read(nameKey, nameRanks)) {
            nameRanks = std::make_shared<std::vector<NameRank>>();
            std::vector<std::vector<Token>> sortedTokensList = tokensList;

            // Optimization for autocomplete - if too many potential matches, remove the token or some matches
            {
                std::uint64_t count = 0;
                for (auto it = sortedTokensList.back().begin(); it != sortedTokensList.back().end(); it++) {
                    count += it->count;
                    if (count > MAX_NAME_MATCH_COUNTER) {
                        if (it != sortedTokensList.back().begin()) {
                            sortedTokensList.back().erase(it);
                        }
                        break;
                    }
                }
            }

            // Drop tokens after pos 32 - sqlite supports only up to 64 joins
            if (sortedTokensList.size() > 32) {
                sortedTokensList.erase(sortedTokensList.begin() + 32, sortedTokensList.end());
            }

            // Sort tokens by increasing name counts
            std::sort(sortedTokensList.begin(), sortedTokensList.end(), [](const std::vector<Token>& tokens1, const std::vector<Token>& tokens2) {
                std::uint64_t count1 = std::accumulate(tokens1.begin(), tokens1.end(), std::uint64_t(0), [](std::uint64_t sum, const Token& token) { return sum + token.count; });
                std::uint64_t count2 = std::accumulate(tokens2.begin(), tokens2.end(), std::uint64_t(0), [](std::uint64_t sum, const Token& token) { return sum + token.count; });
                return count1 < count2;
            });

            // Select names based on tokens
            std::vector<std::string> sqlTables;
            std::vector<std::string> sqlFilters;
            for (const std::vector<Token>& tokens : sortedTokensList) {
                std::string values;
                for (const Token& token : tokens) {
                    values += (values.empty() ? "" : ",") + boost::lexical_cast<std::string>(token.id);
                }
                std::string tableName = "nt" + boost::lexical_cast<std::string>(sqlFilters.size());
                sqlTables.push_back(tableName);
                std::string sqlFilter = tableName + ".token_id IN (" + values + ") AND " + tableName + ".lang IS " + sqlTables.front() + ".lang";
                sqlFilters.push_back(sqlFilters.empty() ? sqlFilter : tableName + ".name_id=" + sqlTables.front() + ".name_id AND " + sqlFilter);
            }

            std::string sql = "SELECT DISTINCT n.id, n.name, n.lang, n.type, n.entitycount FROM (SELECT nt0.name_id, nt0.lang FROM ";
            for (std::size_t i = 0; i < sqlTables.size(); i++) {
                sql += (i > 0 ? " CROSS JOIN " : "") + std::string("nametokens ") + sqlTables[i];
            }
            sql += " WHERE ";
            for (std::size_t i = 0; i < sqlFilters.size(); i++) {
                sql += (i > 0 ? " AND " : "") + std::string("(") + sqlFilters[i] + ")";
            }
            sql += ") nt CROSS JOIN names n WHERE n.id=nt.name_id AND n.lang IS nt.lang AND COALESCE(n.lang, '') IN ('" + escapeSQLValue(_language) + "', '') ORDER BY LENGTH(n.name) ASC LIMIT " + boost::lexical_cast<std::string>(ENTITY_QUERY_LIMIT);

            std::vector<std::shared_ptr<Name>> names;
            std::string namesKey = query.database->id + std::string(1, 0) + sql;
            if (!_nameCache.read(namesKey, names)) {
                sqlite3pp::query sqlQuery(*query.database->db, sql.c_str());

                for (auto qit = sqlQuery.begin(); qit != sqlQuery.end(); qit++) {
                    auto name = std::make_shared<Name>();
                    name->id = qit->get<std::uint64_t>(0);
                    name->name = qit->get<const char*>(1);
                    name->lang = qit->get<const char*>(2) ? qit->get<const char*>(2) : "";
                    name->type = static_cast<FieldType>(qit->get<int>(3));
                    name->count = qit->get<std::uint64_t>(4);

                    sqlite3pp::query sqlQuery2(*query.database->db, "SELECT t.token, t.idf FROM tokens t, nametokens nt WHERE t.id=nt.token_id AND nt.name_id=:nameId");
                    sqlQuery2.bind(":nameId", name->id);
                    for (auto qit2 = sqlQuery2.begin(); qit2 != sqlQuery2.end(); qit2++) {
                        std::string nameToken = qit2->get<const char*>(0);
                        float idf = static_cast<float>(qit2->get<double>(1));
                        name->tokenIDFs.emplace_back(nameToken, idf);
                    }

                    names.push_back(std::move(name));
                }
                names.shrink_to_fit();

                _nameQueryCounter++;
                _nameCache.put(namesKey, names);
            }

            // Match names, use binary search for fast merging
            if (!names.empty()) {
                nameRanks->reserve(names.size());
                for (const std::shared_ptr<Name>& name : names) {
                    float rank = calculateNameRank(query, name->name, matchName, name->tokenIDFs);
                    float threshold = (name->type == FieldType::HOUSENUMBER ? MIN_HOUSENUMBER_MATCH_THRESHOLD : MIN_MATCH_THRESHOLD);
                    if (rank >= threshold) {
                        nameRanks->push_back(NameRank { name, rank });
                    }
                }

                // Sort the results by decreasing ranks
                std::sort(nameRanks->begin(), nameRanks->end(), [](const NameRank& nameRank1, const NameRank& nameRank2) {
                    return nameRank1.rank > nameRank2.rank;
                });
                for (std::size_t i = 1; i < nameRanks->size(); i++) {
                    if (nameRanks->at(i).rank < nameRanks->front().rank * MAX_MATCH_RATIO) {
                        nameRanks->erase(nameRanks->begin() + i, nameRanks->end());
                        break;
                    }
                }
            }
            nameRanks->shrink_to_fit();

            _nameRankCounter++;
            _nameRankCache.put(nameKey, nameRanks);
        }
    }

    void Geocoder::matchEntities(const Query& query, const Options& options, std::vector<Result>& results) const {
        std::vector<std::shared_ptr<std::vector<NameRank>>> filtersList;
        if (!optimizeQueryFilters(query, filtersList)) {
            return;
        }

        // Sort filters by the number of potential matches (ascending)
        std::vector<std::shared_ptr<std::vector<NameRank>>> sortedFiltersList = filtersList;
        std::sort(sortedFiltersList.begin(), sortedFiltersList.end(), [](const std::shared_ptr<std::vector<NameRank>>& nameRanks1, const std::shared_ptr<std::vector<NameRank>>& nameRanks2) {
            std::uint64_t count1 = std::accumulate(nameRanks1->begin(), nameRanks1->end(), std::uint64_t(0), [](std::uint64_t sum, const NameRank& nameRank) { return sum + nameRank.name->count; });
            std::uint64_t count2 = std::accumulate(nameRanks2->begin(), nameRanks2->end(), std::uint64_t(0), [](std::uint64_t sum, const NameRank& nameRank) { return sum + nameRank.name->count; });
            return count1 < count2;
        });

        // Find which fields are potentially present in the query
        std::uint32_t typeMask = 1 << static_cast<int>(FieldType::HOUSENUMBER);
        for (const std::shared_ptr<std::vector<NameRank>>& nameRanks : filtersList) {
            std::uint32_t mask = std::accumulate(nameRanks->begin(), nameRanks->end(), std::uint32_t(0), [](std::uint32_t mask, const NameRank& nameRank) { return mask | 1 << static_cast<int>(nameRank.name->type); });
            typeMask |= mask;
        }

        // Verify that that the first filter is not too generic; if it is, drop name, street, house mode
        const std::vector<NameRank>& nameRanks1 = *sortedFiltersList.front();
        std::uint64_t count1 = std::accumulate(nameRanks1.begin(), nameRanks1.end(), std::uint64_t(0), [](std::uint64_t sum, const NameRank& nameRank) { return sum + nameRank.name->count; });
        if (count1 > MAX_MATCH_COUNT) {
            typeMask &= ~(1 << static_cast<int>(FieldType::NAME)) & ~(1 << static_cast<int>(FieldType::HOUSENUMBER)) & ~(1 << static_cast<int>(FieldType::STREET));
        }

        // Build SQL filters
        const Database& database = *query.database;
        std::vector<std::string> sqlTables;
        std::vector<std::string> sqlFilters;
        for (const std::shared_ptr<std::vector<NameRank>>& nameRanks : sortedFiltersList) {
            std::string values;
            for (const NameRank& nameRank : *nameRanks) {
                values += (values.empty() ? "" : ",") + boost::lexical_cast<std::string>(nameRank.name->id);
            }
            std::string tableName = "en" + boost::lexical_cast<std::string>(sqlFilters.size());
            sqlTables.push_back(tableName);
            std::string sqlFilter = tableName + ".name_id IN (" + values + ")";
            std::uint32_t mask = std::accumulate(nameRanks->begin(), nameRanks->end(), std::uint32_t(0), [](std::uint32_t mask, const NameRank& nameRank) { return mask | 1 << static_cast<int>(nameRank.name->type); });
            sqlFilters.push_back(sqlFilters.empty() ? sqlFilter : tableName + ".entity_id=" + sqlTables.front() + ".entity_id AND " + sqlFilter);
        }

        // Build final SQL using CROSS JOINs. Use two different strategies: if we can not reduce the number of first entitynames matches to a threshold, do matching from entities filtered by type first
        std::string sql = "SELECT DISTINCT e.id, e.features, e.housenumbers, e.rank FROM ";
        if (count1 > MAX_MATCH_COUNT) {
            sql += "entities e";
            for (std::size_t i = 0; i < sqlTables.size(); i++) {
                sql += " CROSS JOIN entitynames " + sqlTables[i];
            }
        }
        else {
            for (std::size_t i = 0; i < sqlTables.size(); i++) {
                sql += "entitynames " + sqlTables[i] + " CROSS JOIN ";
            }
            sql += "entities e";
        }
        sql += " WHERE ";
        for (std::size_t i = 0; i < sqlFilters.size(); i++) {
            sql += "(" + sqlFilters[i] + ") AND ";
        }

        // Filter out unwanted entities
        std::string values;
        for (std::uint32_t type = 0; (1U << type) <= typeMask; type++) {
            if (!_enabledFilters.empty()) {
                if (std::find(_enabledFilters.begin(), _enabledFilters.end(), static_cast<Address::EntityType>(type)) == _enabledFilters.end()) {
                    continue;
                }
            }
            if ((typeMask & (1 << type)) != 0) {
                values += (values.empty() ? "" : ",") + boost::lexical_cast<std::string>(type);
            }
        }
        sql += "(e.id=" + sqlTables.front() + ".entity_id) AND e.type in (" + values + ") ORDER BY e.type ASC, e.rank DESC LIMIT " + boost::lexical_cast<std::string>(ENTITY_QUERY_LIMIT);

        std::string entityKey = database.id + std::string(1, 0) + sql;
        std::vector<EntityRow> entityRows;
        if (!_entityCache.read(entityKey, entityRows)) {
            sqlite3pp::query sqlQuery(*database.db, sql.c_str());
            for (auto qit = sqlQuery.begin(); qit != sqlQuery.end(); qit++) {
                EntityRow entityRow;
                entityRow.id = qit->get<unsigned int>(0);
                if (qit->get<const void*>(1)) {
                    entityRow.features = std::string(static_cast<const char*>(qit->get<const void*>(1)), qit->column_bytes(1));
                }
                if (qit->get<const void*>(2)) {
                    entityRow.houseNumbers = std::string(static_cast<const char*>(qit->get<const void*>(2)), qit->column_bytes(2));
                }
                entityRow.rank = static_cast<float>(qit->get<std::uint64_t>(3) / query.database->rankScale);

                sqlite3pp::query sqlQuery2(*database.db, "SELECT DISTINCT n.type, n.id FROM entitynames en, names n WHERE en.entity_id=:entityId AND en.name_id=n.id");
                sqlQuery2.bind(":entityId", qit->get<std::uint64_t>(0));
                for (auto qit2 = sqlQuery2.begin(); qit2 != sqlQuery2.end(); qit2++) {
                    EntityName entityName;
                    entityName.type = static_cast<FieldType>(qit2->get<int>(0));
                    entityName.id = qit2->get<std::uint64_t>(1);
                    entityRow.entityNames.push_back(entityName);
                }

                entityRows.push_back(std::move(entityRow));
            }
            entityRows.shrink_to_fit();

            _entityQueryCounter++;
            _entityCache.put(entityKey, entityRows);
            _missingEntityQueryCounter += (entityRows.empty() ? 1 : 0);
        }

        if (entityRows.empty()) {
            return;
        }

        auto mercatorConverter = [&database](const cglib::vec2<double>& pos) {
            return wgs84ToWebMercator(database.origin + pos);
        };

        for (const EntityRow& entityRow : entityRows) {
            EncodingStream houseNumberStream(entityRow.houseNumbers.data(), entityRow.houseNumbers.size());
            AddressInterpolator interpolator(houseNumberStream);

            std::function<void(std::size_t, std::uint32_t, float, unsigned int, std::map<unsigned int, float>&)> findBestMatches;
            findBestMatches = [&](std::size_t index, std::uint32_t mask, float rank, unsigned int elementIndex, std::map<unsigned int, float>& bestMatches) {
                if (index >= query.filtersList.size()) {
                    for (const EntityName& entityName : entityRow.entityNames) {
                        if (!(mask & (1 << static_cast<int>(entityName.type)))) {
                            rank *= EXTRA_FIELD_PENALTY;
                        }
                    }
                    if (!entityRow.houseNumbers.empty() && elementIndex == 0) {
                        rank *= EXTRA_FIELD_PENALTY;
                    }
                    bestMatches[elementIndex] = std::max(rank, bestMatches[elementIndex]);
                    return;
                }

                for (const NameRank& nameRank : *query.filtersList[index]) {
                    if (mask & (1 << static_cast<int>(nameRank.name->type))) {
                        continue;
                    }

                    if (nameRank.name->type == FieldType::HOUSENUMBER) {
                        int houseIndex = interpolator.findAddress(nameRank.name->id); // if not found, interpolator returns -1
                        if (houseIndex != -1) {
                            findBestMatches(index + 1, mask | (1 << static_cast<int>(nameRank.name->type)), rank * nameRank.rank, houseIndex + 1, bestMatches);
                        }
                    }
                    else {
                        auto it = std::find_if(entityRow.entityNames.begin(), entityRow.entityNames.end(), [&nameRank](const EntityName& entityName) {
                            return entityName.id == nameRank.name->id;
                        });
                        if (it != entityRow.entityNames.end()) {
                            findBestMatches(index + 1, mask | (1 << static_cast<int>(nameRank.name->type)), rank * nameRank.rank, elementIndex, bestMatches);
                        }
                    }
                }
                    
                findBestMatches(index + 1, mask, rank * UNMATCHED_FIELD_PENALTY, elementIndex, bestMatches);
            };


            std::map<unsigned int, float> bestMatches;
            findBestMatches(0, 0, 1.0f, 0, bestMatches);

            for (auto it = bestMatches.begin(); it != bestMatches.end(); it++) {
                unsigned int elementIndex = it->first;
                float rank = it->second;

                auto getFeatures = [&]() -> std::vector<Feature> {
                    EncodingStream featureStream(entityRow.features.data(), entityRow.features.size());
                    FeatureReader featureReader(featureStream, mercatorConverter);

                    std::vector<Feature> features;
                    if (elementIndex > 0) {
                        features = interpolator.readAddressesAndFeatures(featureReader).at(elementIndex - 1).second;
                    }
                    else {
                        features = featureReader.readFeatureCollection();
                    }
                    return features;
                };

                // Check that geometry is inside bounds
                if (options.bounds) {
                    bool inside = false;
                    for (const Feature& feature : getFeatures()) {
                        if (std::shared_ptr<Geometry> geometry = feature.getGeometry()) {
                            cglib::bbox2<double> mercatorBounds = geometry->getBounds();
                            cglib::bbox2<double> bounds(webMercatorToWgs84(mercatorBounds.min), webMercatorToWgs84(mercatorBounds.max));
                            if (options.bounds->inside(bounds)) {
                                inside = true;
                                break;
                            }
                        }
                    }
                    if (!inside) {
                        continue;
                    }
                }

                // Create result
                Result result;
                result.database = query.database;
                result.encodedId = (static_cast<std::uint64_t>(elementIndex) << 32) | entityRow.id;
                result.unmatchedTokens = query.tokenList.unmatchedTokens();

                // Set penalty for unmatched fields
                result.matchRank = rank;
                result.matchRank *= std::pow(UNMATCHED_FIELD_PENALTY, query.tokenList.unmatchedTokens());

                // Set entity ranking
                result.entityRank *= entityRow.rank;

                // Do location based ranking
                if (options.location) {
                    float minDist = std::numeric_limits<float>::infinity();
                    for (const Feature& feature : getFeatures()) {
                        if (std::shared_ptr<Geometry> geometry = feature.getGeometry()) {
                            cglib::vec2<double> mercatorMeters = webMercatorMeters(*options.location);
                            cglib::vec2<double> mercatorLocation = wgs84ToWebMercator(*options.location);
                            cglib::vec2<double> point = geometry->calculateNearestPoint(mercatorLocation);
                            cglib::vec2<double> diff = point - mercatorLocation;
                            float dist = static_cast<float>(cglib::length(cglib::vec2<double>(diff(0) * mercatorMeters(0), diff(1) * mercatorMeters(1))));
                            minDist = std::min(minDist, dist);
                        }
                    }

                    float distRank = std::exp(-0.5f * std::pow(minDist / options.locationSigma, 2.0f));
                    result.locationRank *= MIN_LOCATION_RANK + (1.0f - MIN_LOCATION_RANK) * distRank;
                }

                // Early out test
                if (result.totalRank() < MIN_RANK_THRESHOLD) {
                    continue;
                }

                // Check if the same result is already stored
                auto resultIt = std::find_if(results.begin(), results.end(), [&result](const Result& result2) {
                    return result.encodedId == result2.encodedId;
                    });
                if (resultIt != results.end()) {
                    if (resultIt->totalRank() >= result.totalRank()) {
                        continue; // if we have stored the row with better ranking, ignore current
                    }
                    results.erase(resultIt); // erase the old match, as the new match is better
                }

                // Find position for the result
                resultIt = std::upper_bound(results.begin(), results.end(), result, [](const Result& result1, const Result& result2) {
                    return result1.totalRank() > result2.totalRank();
                    });
                if (!(resultIt == results.end() && results.size() == _maxResults)) {
                    results.insert(resultIt, result);

                    // Drop results that have too low rankings
                    while (!results.empty()) {
                        if (results.front().totalRank() * MAX_RANK_RATIO <= results.back().totalRank() && results.back().totalRank() >= MIN_RANK_THRESHOLD) {
                            break;
                        }
                        results.pop_back();
                    }
                }
            }
        }
    }

    bool Geocoder::optimizeQueryFilters(const Query& query, std::vector<std::shared_ptr<std::vector<NameRank>>>& filtersList) const {
        std::function<bool(const std::vector<std::uint32_t>&)> validAssignment;
        validAssignment = [&validAssignment](const std::vector<std::uint32_t>& masks) {
            if (masks.empty()) {
                return true;
            }
            if (std::any_of(masks.begin(), masks.end(), [](std::uint32_t mask) { return mask == 0; })) {
                return false;
            }
            for (std::size_t mask = 1; mask <= masks[0]; mask <<= 1) {
                std::vector<std::uint32_t> submasks(masks.begin() + 1, masks.end());
                std::for_each(submasks.begin(), submasks.end(), [mask](std::uint32_t& submask) { submask &= ~mask; });
                if (validAssignment(submasks)) {
                    return true;
                }
            }
            return false;
        };

        // Calculate potentially valid type masks for each name
        std::vector<std::uint32_t> validMasks;
        validMasks.reserve(query.filtersList.size());
        for (const std::shared_ptr<std::vector<NameRank>>& nameRanks : query.filtersList) {
            std::uint32_t validMask = std::accumulate(nameRanks->begin(), nameRanks->end(), std::uint32_t(0), [](std::uint32_t mask, const NameRank& nameRank) { return mask | 1 << static_cast<int>(nameRank.name->type); });
            validMasks.push_back(validMask);
        }

        // Now iteratively try to remove impossible combinations. For example, for ({Street}, {Street, Region}) we can replace this with ({Street}, {Region})
        // Also, we assume that house numbers can be only present if there is a street present. Thus ({HouseNumber, Name}, {Country}) can be replaced with ({Name}, {Country})
        bool progress;
        do {
            progress = false;
            for (std::size_t i = 0; i < query.filtersList.size(); i++) {
                std::uint32_t validMask = 0;
                for (std::uint32_t mask = 1; mask <= validMasks[i]; mask <<= 1) {
                    if (validMasks[i] & mask) {
                        std::vector<std::uint32_t> masks = validMasks;
                        for (std::size_t j = 0; j < masks.size(); j++) {
                            masks[j] = (i == j ? mask : masks[j] & ~mask);
                        }
                        if (validAssignment(masks)) {
                            validMask |= mask;
                        }
                    }
                }

                bool streetType = false;
                for (std::size_t j = 0; j < query.filtersList.size(); j++) {
                    if (i != j && (validMasks[j] & (1 << static_cast<int>(FieldType::STREET))) != 0) {
                        streetType = true;
                        break;
                    }
                }
                if (!streetType) {
                    validMask &= ~(1 << static_cast<int>(FieldType::HOUSENUMBER));
                }

                if (validMask != validMasks[i]) {
                    validMasks[i] = validMask;
                    progress = true;
                }
            }
        } while (progress);

        // Now simplify the query filters according to new valid type masks by removing impossible records.
        // We will also remove housenumbers, as these are not included in entitynames table.
        filtersList.reserve(query.filtersList.size());
        bool nameType = false;
        for (std::size_t i = 0; i < query.filtersList.size(); i++) {
            if (validMasks[i] & (1 << static_cast<int>(FieldType::NAME))) {
                nameType = true;
            }
            if (validMasks[i] & (1 << static_cast<int>(FieldType::HOUSENUMBER))) {
                continue;
            }

            auto nameRanks = std::make_shared<std::vector<NameRank>>();
            nameRanks->reserve(query.filtersList[i]->size());
            for (const NameRank& nameRank : *query.filtersList[i]) {
                if (validMasks[i] & (1 << static_cast<int>(nameRank.name->type))) {
                    nameRanks->push_back(nameRank);
                }
            }
            if (nameRanks->empty()) {
                return false;
            }
            filtersList.push_back(std::move(nameRanks));
        }
        return !filtersList.empty();
    }

    float Geocoder::calculateNameRank(const Query& query, const std::string& name, const std::string& queryName, const std::vector<std::pair<std::string, float>>& tokenIDFs) const {
        float rank = 1.0f;
        std::string nameKey = query.database->id + std::string(1, 0) + name + std::string(1, 0) + queryName;
        if (!_nameMatchCache.read(nameKey, rank)) {
            auto getTokenRank = [&tokenIDFs, &query](const unistring::unistring& token) {
                std::string translatedToken = unistring::to_utf8string(getTranslatedToken(token, query.database->translationTable));
                auto it = std::find_if(tokenIDFs.begin(), tokenIDFs.end(), [&translatedToken](const std::pair<std::string, float>& tokenIDF) {
                    return tokenIDF.first == translatedToken;
                });
                if (it != tokenIDFs.end()) {
                    return it->second;
                }
                return 1.0f;
            };

            StringMatcher<unistring::unistring> matcher(getTokenRank);
            matcher.setMaxDist(MAX_STRINGMATCH_DIST);
            matcher.setTranslationTable(query.database->translationTable, TRANSLATION_EXTRA_PENALTY);
            if (_autocomplete) {
                matcher.setWildcardChar('%', AUTOCOMPLETE_EXTRA_CHAR_PENALTY);
            }
            rank = matcher.calculateRating(unistring::to_lower(unistring::to_unistring(queryName)), unistring::to_lower(unistring::to_unistring(name)));

            _nameMatchCounter++;
            _nameMatchCache.put(nameKey, rank);
        }
        return rank;
    }

    cglib::vec2<double> Geocoder::getOrigin(sqlite3pp::database& db) {
        sqlite3pp::query query(db, "SELECT value FROM metadata WHERE name='origin'");
        for (auto qit = query.begin(); qit != query.end(); qit++) {
            std::string value = qit->get<const char*>(0);

            std::vector<std::string> origin;
            boost::split(origin, value, boost::is_any_of(","), boost::token_compress_off);
            return cglib::vec2<double>(boost::lexical_cast<double>(origin.at(0)), boost::lexical_cast<double>(origin.at(1)));
        }
        return cglib::vec2<double>(0, 0);
    }

    cglib::bbox2<double> Geocoder::getBounds(sqlite3pp::database& db) {
        sqlite3pp::query query(db, "SELECT value FROM metadata WHERE name='bounds'");
        for (auto qit = query.begin(); qit != query.end(); qit++) {
            std::string value = qit->get<const char*>(0);

            std::vector<std::string> origin;
            boost::split(origin, value, boost::is_any_of(","), boost::token_compress_off);
            return cglib::bbox2<double>(
                cglib::vec2<double>(boost::lexical_cast<double>(origin.at(0)), boost::lexical_cast<double>(origin.at(1))),
                cglib::vec2<double>(boost::lexical_cast<double>(origin.at(2)), boost::lexical_cast<double>(origin.at(3)))
            );
        }
        return cglib::bbox2<double>(cglib::vec2<double>(-180, -90), cglib::vec2<double>(180, 90));
    }

    double Geocoder::getRankScale(sqlite3pp::database& db) {
        sqlite3pp::query query(db, "SELECT value FROM metadata WHERE name='rank_scale'");
        for (auto qit = query.begin(); qit != query.end(); qit++) {
            std::string value = qit->get<const char*>(0);
            return boost::lexical_cast<double>(value);
        }
        return 32767.0;
    }

    std::unordered_map<unistring::unichar_t, unistring::unistring> Geocoder::getTranslationTable(sqlite3pp::database& db) {
        sqlite3pp::query query(db, "SELECT value FROM metadata WHERE name='translation_table'");
        for (auto qit = query.begin(); qit != query.end(); qit++) {
            std::string value = qit->get<const char*>(0);

            std::vector<std::string> translationVector;
            boost::split(translationVector, value, boost::is_any_of(","), boost::token_compress_off);
            std::unordered_map<unistring::unichar_t, unistring::unistring> translationTable;
            for (const std::string& translation : translationVector) {
                unistring::unistring uniTranslation = unistring::to_unistring(translation);
                if (uniTranslation.size() >= 2 && uniTranslation[1] == ':') {
                    translationTable[uniTranslation[0]] = uniTranslation.substr(2);
                }
            }
            return translationTable;
        }
        return std::unordered_map<unistring::unichar_t, unistring::unistring>();
    }

    unistring::unistring Geocoder::getTranslatedToken(const unistring::unistring& token, const std::unordered_map<unistring::unichar_t, unistring::unistring>& translationTable) {
        unistring::unistring translatedToken;
        translatedToken.reserve(token.size());
        for (unistring::unichar_t c : token) {
            auto it = translationTable.find(c);
            if (it != translationTable.end()) {
                translatedToken += it->second;
            }
            else {
                translatedToken.append(1, c);
            }
        }
        return translatedToken;
    }
} }
