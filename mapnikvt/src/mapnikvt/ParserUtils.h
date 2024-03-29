/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_PARSERUTILS_H_
#define _CARTO_MAPNIKVT_PARSERUTILS_H_

#include "Value.h"
#include "Expression.h"
#include "vt/Color.h"
#include "vt/Styles.h"

#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

namespace carto::mvt {
    class ParserException : public std::runtime_error {
    public:
        explicit ParserException(const std::string& msg) : runtime_error(msg), _message(msg), _source() { }
        explicit ParserException(const std::string& msg, const std::string& source) : runtime_error(msg + ": " + source), _message(msg), _source(source) { }

        const std::string& message() const { return _message; }
        const std::string& source() const { return _source; }

    private:
        std::string _message;
        std::string _source;
    };

    vt::LineCapMode parseLineCapMode(const std::string& str);
    vt::LineJoinMode parseLineJoinMode(const std::string& str);
    vt::CompOp parseCompOp(const std::string& str);
    vt::LabelOrientation parseLabelOrientation(const std::string& str);
    vt::Color parseColor(const std::string& str);
    Value parseValue(const std::string& str);
    Expression parseExpression(const std::string& str, bool stringExpr);
}

#endif
