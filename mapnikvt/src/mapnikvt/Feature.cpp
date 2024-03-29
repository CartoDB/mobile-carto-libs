#include "Feature.h"

#include <algorithm>

namespace carto::mvt {
    bool FeatureData::getVariable(const std::string& name, Value& value) const {
        auto it = std::find_if(_variables.begin(), _variables.end(), [&name](const std::pair<std::string, Value>& var) { return var.first == name; });
        if (it == _variables.end()) {
            return false;
        }
        value = it->second;
        return true;
    }
}
