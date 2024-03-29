/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_MAPNIKVT_EXPRESSIONGENERATOR_H_
#define _CARTO_MAPNIKVT_EXPRESSIONGENERATOR_H_

#include "Value.h"
#include "Expression.h"
#include "Predicate.h"
#include "Transform.h"
#include "ValueGenerator.h"

#include <memory>
#include <functional>

#include <boost/phoenix.hpp>
#include <boost/spirit/include/karma.hpp>

namespace carto::mvt {
    namespace exprgenimpl {
        template <typename OutputIterator, bool StringExpression>
        struct Grammar : boost::spirit::karma::grammar<OutputIterator, Expression()> {
            Grammar() : Grammar::base_type(StringExpression ? stringExpression : genericExpression) {
                using namespace boost;
                using namespace boost::spirit;
                using karma::_pass;
                using karma::_val;
                using karma::_1;
                using karma::_2;
                using karma::_3;
                using karma::_4;
                using karma::_5;
                using karma::_6;

                esc_char.add('\a', "\\a")('\b', "\\b")('\f', "\\f")('\n', "\\n")
                            ('\r', "\\r")('\t', "\\t")('\v', "\\v")('\\', "\\\\")
                            ('\'', "\\\'")('"', "\\\"")('[', "\\[")(']', "\\]")('{', "\\{")('}', "\\}");

                string %= *(esc_char | karma::print | ("\\x0" << octet) [_pass = _1 >= 0x00 && _1 <= 0x0f] | "\\x" << octet);

                stringExpression =
                      string                               [_pass = phoenix::bind(&getString, _val, _1)]
                    | (stringExpression << stringExpression) [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::CONCAT, _val, _1, _2)]
                    | ('[' << stringExpression << ']' )    [_pass = phoenix::bind(&getVariableExpression, _val, _1)]
                    | ('{' << expression << '}')           [_1 = _val]
                    ;

                genericExpression =
                    expression                             [_1 = _val]
                    ;

                expression =
                      ('(' << term0 << " ? " << expression << " : " << expression << ')') [_pass = phoenix::bind(&getTertiaryExpression, TertiaryExpression::Op::CONDITIONAL, _val, _1, _2, _3)]
                    | term0                                [_1 = _val]
                    ;

                term0 =
                      (term0 << " and " << term1)          [_pass = phoenix::bind(&getAndPredicate, _val, _1, _2)]
                    | (term0 << " or "  << term1)          [_pass = phoenix::bind(&getOrPredicate,  _val, _1, _2)]
                    | term1                                [_1 = _val]
                    ;

                term1 =
                      (term1 << "<>" << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::NEQ, _val, _1, _2)]
                    | (term1 << "<=" << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::LTE, _val, _1, _2)]
                    | (term1 << ">=" << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::GTE, _val, _1, _2)]
                    | (term1 << "!=" << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::NEQ, _val, _1, _2)]
                    | (term1 << '<'  << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::LT,  _val, _1, _2)]
                    | (term1 << '>'  << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::GT,  _val, _1, _2)]
                    | (term1 << '='  << term2)             [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::EQ,  _val, _1, _2)]
                    | term2                                [_1 = _val]
                    ;

                term2 =
                      (term2 << '+' << term3)              [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::ADD, _val, _1, _2)]
                    | (term2 << '-' << term3)              [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::SUB, _val, _1, _2)]
                    | term3                                [_1 = _val]
                    ;

                term3 =
                      (term3 << '*' << unary)              [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::MUL, _val, _1, _2)]
                    | (term3 << '/' << unary)              [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::DIV, _val, _1, _2)]
                    | (term3 << '%' << unary)              [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::MOD, _val, _1, _2)]
                    | unary                                [_1 = _val]
                    ;

                unary =
                      ('-' << unary)                       [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::NEG, _val, _1)]
                    | ('!' << unary)                       [_pass = phoenix::bind(&getNotPredicate, _val, _1)]
                    | (karma::lit("exp") << '(' << expression << ')') [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::EXP, _val, _1)]
                    | (karma::lit("log") << '(' << expression << ')') [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::LOG, _val, _1)]
                    | postfix                              [_1 = _val]
                    ;

                postfix =
                      (postfix << '.' << karma::lit("length"))                              [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::LENGTH, _val, _1)]
                    | (postfix << '.' << karma::lit("uppercase"))                           [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::UPPER,  _val, _1)]
                    | (postfix << '.' << karma::lit("lowercase"))                           [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::LOWER,  _val, _1)]
                    | (postfix << '.' << karma::lit("capitalize"))                          [_pass = phoenix::bind(&getUnaryExpression, UnaryExpression::Op::CAPITALIZE, _val, _1)]
                    | (postfix << '.' << karma::lit("concat")  << '(' << expression << ')') [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::CONCAT, _val, _1, _2)]
                    | (postfix << '.' << karma::lit("match")   << '(' << expression << ')') [_pass = phoenix::bind(&getComparisonPredicate, ComparisonPredicate::Op::MATCH, _val, _1, _2)]
                    | (postfix << '.' << karma::lit("replace") << '(' << expression << ',' << expression << ')') [_pass = phoenix::bind(&getTertiaryExpression, TertiaryExpression::Op::REPLACE, _val, _1, _2, _3)]
                    | factor                                                                [_1 = _val]
                    ;

                factor =
                      constant                          [_pass = phoenix::bind(&getConstant, _val, _1)]
                    | (karma::lit("pow" )   << '(' << expression << ',' << expression << ')')       [_pass = phoenix::bind(&getBinaryExpression, BinaryExpression::Op::POW, _val, _1, _2)]
                    | (karma::lit("step")   << '(' << expression << ',' << (constant % ',') << ')') [_pass = phoenix::bind(&getInterpolateExpression, InterpolateExpression::Method::STEP, _val, _1, _2)]
                    | (karma::lit("linear") << '(' << expression << ',' << (constant % ',') << ')') [_pass = phoenix::bind(&getInterpolateExpression, InterpolateExpression::Method::LINEAR, _val, _1, _2)]
                    | (karma::lit("cubic")  << '(' << expression << ',' << (constant % ',') << ')') [_pass = phoenix::bind(&getInterpolateExpression, InterpolateExpression::Method::CUBIC, _val, _1, _2)]
                    | (karma::lit("matrix") << '(' << expression << ',' << expression << ',' << expression << ',' << expression << ',' << expression << ',' << expression << ')') [_pass = phoenix::bind(&getMatrixTransformExpression, _val, _1, _2, _3, _4, _5, _6)]
                    | (karma::lit("translate") << '(' << expression << ',' << expression << ')') [_pass = phoenix::bind(&getTranslateTransformExpression, _val, _1, _2)]
                    | (karma::lit("rotate") << '(' << expression << ',' << expression << ',' << expression << ')') [_pass = phoenix::bind(&getRotateTransformExpression, _val, _2, _3, _1)]
                    | (karma::lit("scale")  << '(' << expression << ',' << expression << ')') [_pass = phoenix::bind(&getScaleTransformExpression, _val, _1, _2)]
                    | (karma::lit("skewx")  << '(' << expression << ')') [_pass = phoenix::bind(&getSkewXTransformExpression, _val, _1)]
                    | (karma::lit("skewy")  << '(' << expression << ')') [_pass = phoenix::bind(&getSkewYTransformExpression, _val, _1)]
                    | predicate                         [_pass = phoenix::bind(&getExpressionPredicate, _val, _1)]
                    | ('[' << stringExpression << ']')  [_pass = phoenix::bind(&getVariableExpression, _val, _1)]
                    | ('(' << expression << ')')        [_1 = _val]
                    ;

                predicate =
                      expression                        [_pass = phoenix::bind(&getPredicateExpression, _val, _1)]
                    ;
            }

            ValueGeneratorGrammar<OutputIterator> constant;
            boost::spirit::karma::uint_generator<unsigned char, 16> octet;
            boost::spirit::karma::symbols<char, const char*> esc_char;
            boost::spirit::karma::rule<OutputIterator, std::string()> string;
            boost::spirit::karma::rule<OutputIterator, Expression()> stringExpression, genericExpression;
            boost::spirit::karma::rule<OutputIterator, Expression()> expression, term0, term1, term2, term3, unary, postfix, factor;
            boost::spirit::karma::rule<OutputIterator, Predicate()> predicate;

        private:
            static bool getString(const Expression& expr, std::string& str) {
                if (auto val = std::get_if<Value>(&expr)) {
                    if (auto strVal = std::get_if<std::string>(val)) {
                        str = *strVal;
                        return true;
                    }
                }
                return false;
            }

            static bool getConstant(const Expression& expr, Value& val1) {
                if (auto val = std::get_if<Value>(&expr)) {
                    val1 = *val;
                    return true;
                }
                return false;
            }

            static bool getExpressionPredicate(const Expression& expr, Predicate& pred1) {
                if (auto pred = std::get_if<Predicate>(&expr)) {
                    pred1 = *pred;
                    return true;
                }
                return false;
            }

            static bool getVariableExpression(const Expression& expr, Expression& expr1) {
                if (auto varExpr = std::get_if<std::shared_ptr<VariableExpression>>(&expr)) {
                    expr1 = (*varExpr)->getVariableExpression();
                    return true;
                }
                return false;
            }

            static bool getPredicateExpression(const Predicate& pred, Expression& expr1) {
                if (auto exprPred = std::get_if<std::shared_ptr<ExpressionPredicate>>(&pred)) {
                    expr1 = (*exprPred)->getExpression();
                    return true;
                }
                return false;
            }

            static bool getNotPredicate(const Expression& expr, Expression& expr1) {
                if (auto pred = std::get_if<Predicate>(&expr)) {
                    if (auto notPred = std::get_if<std::shared_ptr<NotPredicate>>(pred)) {
                        expr1 = (*notPred)->getPredicate();
                        return true;
                    }
                }
                return false;
            }

            static bool getOrPredicate(const Expression& expr, Expression& expr1, Expression& expr2) {
                if (auto pred = std::get_if<Predicate>(&expr)) {
                    if (auto orPred = std::get_if<std::shared_ptr<OrPredicate>>(pred)) {
                        expr1 = (*orPred)->getPredicate1();
                        expr2 = (*orPred)->getPredicate2();
                        return true;
                    }
                }
                return false;
            }

            static bool getAndPredicate(const Expression& expr, Expression& expr1, Expression& expr2) {
                if (auto pred = std::get_if<Predicate>(&expr)) {
                    if (auto andPred = std::get_if<std::shared_ptr<AndPredicate>>(pred)) {
                        expr1 = (*andPred)->getPredicate1();
                        expr2 = (*andPred)->getPredicate2();
                        return true;
                    }
                }
                return false;
            }

            static bool getComparisonPredicate(ComparisonPredicate::Op op, const Expression& expr, Expression& expr1, Expression& expr2) {
                if (auto pred = std::get_if<Predicate>(&expr)) {
                    if (auto comparisonPred = std::get_if<std::shared_ptr<ComparisonPredicate>>(pred)) {
                        if ((*comparisonPred)->getOp() == op) {
                            expr1 = (*comparisonPred)->getExpression1();
                            expr2 = (*comparisonPred)->getExpression2();
                            return true;
                        }
                    }
                }
                return false;
            }

            static bool getUnaryExpression(UnaryExpression::Op op, const Expression& expr, Expression& expr1) {
                if (auto unaryExpr = std::get_if<std::shared_ptr<UnaryExpression>>(&expr)) {
                    if ((*unaryExpr)->getOp() == op) {
                        expr1 = (*unaryExpr)->getExpression();
                        return true;
                    }
                }
                return false;
            }

            static bool getBinaryExpression(BinaryExpression::Op op, const Expression& expr, Expression& expr1, Expression& expr2) {
                if (auto binaryExpr = std::get_if<std::shared_ptr<BinaryExpression>>(&expr)) {
                    if ((*binaryExpr)->getOp() == op) {
                        expr1 = (*binaryExpr)->getExpression1();
                        expr2 = (*binaryExpr)->getExpression2();
                        return true;
                    }
                }
                return false;
            }

            static bool getTertiaryExpression(TertiaryExpression::Op op, const Expression& expr, Expression& expr1, Expression& expr2, Expression& expr3) {
                if (auto tertiaryExpr = std::get_if<std::shared_ptr<TertiaryExpression>>(&expr)) {
                    if ((*tertiaryExpr)->getOp() == op) {
                        expr1 = (*tertiaryExpr)->getExpression1();
                        expr2 = (*tertiaryExpr)->getExpression2();
                        expr3 = (*tertiaryExpr)->getExpression3();
                        return true;
                    }
                }
                return false;
            }

            static bool getInterpolateExpression(InterpolateExpression::Method method, const Expression& expr, Expression& timeExpr, std::vector<Value>& keyFrames) {
                if (auto interpolateExpr = std::get_if<std::shared_ptr<InterpolateExpression>>(&expr)) {
                    if ((*interpolateExpr)->getMethod() == method) {
                        timeExpr = (*interpolateExpr)->getTimeExpression();
                        keyFrames = (*interpolateExpr)->getKeyFrames();
                        return true;
                    }
                }
                return false;
            }

            static bool getMatrixTransformExpression(const Expression& expr, Expression& a, Expression& b, Expression& c, Expression& d, Expression& e, Expression& f) {
                if (auto transExpr = std::get_if<std::shared_ptr<TransformExpression>>(&expr)) {
                    if (auto matrixTransform = std::get_if<MatrixTransform>(&(*transExpr)->getTransform())) {
                        const std::array<Expression, 6>& values = matrixTransform->getValues();
                        a = values[0];
                        b = values[1];
                        c = values[2];
                        d = values[3];
                        e = values[4];
                        f = values[5];
                        return true;
                    }
                }
                return false;
            }

            static bool getTranslateTransformExpression(const Expression& expr, Expression& dx, Expression& dy) {
                if (auto transExpr = std::get_if<std::shared_ptr<TransformExpression>>(&expr)) {
                    if (auto translateTransform = std::get_if<TranslateTransform>(&(*transExpr)->getTransform())) {
                        dx = translateTransform->getDeltaX();
                        dy = translateTransform->getDeltaY();
                        return true;
                    }
                }
                return false;
            }

            static bool getRotateTransformExpression(const Expression& expr, Expression& x, Expression& y, Expression& angle) {
                if (auto transExpr = std::get_if<std::shared_ptr<TransformExpression>>(&expr)) {
                    if (auto rotateTransform = std::get_if<RotateTransform>(&(*transExpr)->getTransform())) {
                        x = rotateTransform->getOriginX();
                        y = rotateTransform->getOriginY();
                        angle = rotateTransform->getAngle();
                        return true;
                    }
                }
                return false;
            }

            static bool getScaleTransformExpression(const Expression& expr, Expression& sx, Expression& sy) {
                if (auto transExpr = std::get_if<std::shared_ptr<TransformExpression>>(&expr)) {
                    if (auto scaleTransform = std::get_if<ScaleTransform>(&(*transExpr)->getTransform())) {
                        sx = scaleTransform->getScaleX();
                        sy = scaleTransform->getScaleY();
                        return true;
                    }
                }
                return false;
            }

            static bool getSkewXTransformExpression(const Expression& expr, Expression& angle) {
                if (auto transExpr = std::get_if<std::shared_ptr<TransformExpression>>(&expr)) {
                    if (auto skewTransform = std::get_if<SkewXTransform>(&(*transExpr)->getTransform())) {
                        angle = skewTransform->getAngle();
                        return true;
                    }
                }
                return false;
            }

            static bool getSkewYTransformExpression(const Expression& expr, Expression& angle) {
                if (auto transExpr = std::get_if<std::shared_ptr<TransformExpression>>(&expr)) {
                    if (auto skewTransform = std::get_if<SkewYTransform>(&(*transExpr)->getTransform())) {
                        angle = skewTransform->getAngle();
                        return true;
                    }
                }
                return false;
            }
        };
    }

    template <typename Iterator> using ExpressionGeneratorGrammar = exprgenimpl::Grammar<Iterator, false>;
    template <typename Iterator> using StringExpressionGeneratorGrammar = exprgenimpl::Grammar<Iterator, true>;
}

#endif
