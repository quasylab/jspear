/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *              Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.unicam.quasylab.jspear.speclang.parsing;

import it.unicam.quasylab.jspear.speclang.types.JSpearType;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;

public class ParseUtil {
    private static final String MISSING_VARIABLE_DECLARATION = "No variable is declared in the model.";
    private static final String DUPLICATED_VARIABLE_DECLARATION = "Duplicated variables declaration.";
    private static final String DUPLICATED_ENVIRONMENT_DECLARATION = "Duplicated environment declaration.";
    private static final String DUPLICATED_CONTROLLER_DECLARATION = "Duplicated environment declaration.";
    private static final String DUPLICATED_SYMBOL_MESSAGE = "Duplicated definition of symbol %s: first at line %d char %d then at line %d char %d.";
    private static final String TYPE_ERROR_MESSAGE = "Type error at line %d char %d: expected %s is %s.";
    private static final String EXPECTED_NUMERICAL_TYPE_MESSAGE = "A numerical type is expected at line %d char %d: it was %s.";
    private static final String IS_NOT_A_FUNCTION_MESSAGE = "Function %s invoked at line %d char %d is unknown.";
    private static final String ILLEGAL_NUMBER_OF_ARGUMENTS_MESSAGE = "Illegal number of parameters for function %s at line %d char %d: expected %d are %d.";
    private static final String UNKNOWN_SYMBOL_MESSAGE = "Symbol %s used at line %d char %d is unknown.";
    private static final String ILLEGAL_USE_OF_NAME_MESSAGE = "Illegal use of name %s at line %d char %d.";
    private static final String ILLEGAL_USE_OF_ARRAY_SYNTAX = "Illegal use of %s as an array at line %d char %d.";
    private static final String UNKNOWN_STATE_MESSAGE = "State %s used at line %d char %d is unknown.";
    private static final String UNKNOWN_VARIABLE_MESSAGE = "Variable %s used at line %d char %d is unknown.";
    private static final String ILLEGAL_RANGE_INTERVAL_MESSAGE = "Illegal usage of range declaration for variable %s at line %d char %d.";
    private static final String RANGE_INTERVAL_IS_MISSING_MESSAGE = "Range declaration is missing for variable %s at line %d char %d.";
    private static final String UNKNOWN_TYPE_MESSAGE = "Type %s used at line %d char %d is unknown.";
    private static final String ILLEGAL_USE_OF_RANDOM_EXPRESSION = "Illegal use of random expression at line %d char %d.";

    public static ParseError missingVariablesDeclaration() {
        return new ParseError(MISSING_VARIABLE_DECLARATION,0,0);
    }

    public static ParseError duplicatedVariablesDeclaration(Token token) {
        return new ParseError(DUPLICATED_VARIABLE_DECLARATION, token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError duplicatedEnvironmentDeclaration() {
        return new ParseError(DUPLICATED_ENVIRONMENT_DECLARATION,0,0);
    }

    public static ParseError duplicatedControllerDeclaration() {
        return new ParseError(DUPLICATED_CONTROLLER_DECLARATION,0,0);
    }

    public static ParseError duplicatedSymbol(String name, ParserRuleContext first, ParserRuleContext other) {
        return new ParseError(getDuplicatedSymbolMessage(name,
                first.start.getLine(),
                first.start.getCharPositionInLine(), other.start.getLine(), other.start.getCharPositionInLine()
        ), other.start.getLine(), other.start.getCharPositionInLine());
    }

    private static String getDuplicatedSymbolMessage(String name, int firstLine, int firstCharPositionInLine,
                                                     int otherLine, int otherCharPositionInLine) {
        return String.format(DUPLICATED_SYMBOL_MESSAGE, name, firstLine, firstCharPositionInLine, otherLine, otherCharPositionInLine);
    }

    public static ParseError typeError(JSpearType expected, JSpearType actual, Token expressionToken) {
        return new ParseError(getTypeErrorMessage(expected, actual, expressionToken), expressionToken.getLine(), expressionToken.getCharPositionInLine());
    }

    private static String getTypeErrorMessage(JSpearType expected, JSpearType actual, Token expressionToken) {
        return String.format(TYPE_ERROR_MESSAGE, expressionToken.getLine(), expressionToken.getCharPositionInLine(), expected, actual);
    }

    public static ParseError expectedNumericalType(JSpearType actual, Token start) {
        return new ParseError(getExpectedNumericalTypeMessage(actual, start), start.getLine(), start.getCharPositionInLine());
    }

    private static String getExpectedNumericalTypeMessage(JSpearType actual, Token start) {
        return String.format(EXPECTED_NUMERICAL_TYPE_MESSAGE, start.getLine(), start.getCharPositionInLine(), actual);
    }

    public static ParseError isNotAFunction(Token callToken) {
        return new ParseError(getIsNotAFunctionMessage(callToken), callToken.getLine(), callToken.getCharPositionInLine());
    }

    private static String getIsNotAFunctionMessage(Token callToken) {
        return String.format(IS_NOT_A_FUNCTION_MESSAGE, callToken.getText(), callToken.getLine(), callToken.getCharPositionInLine());
    }

    public static ParseError illegalNumberOfArguments(Token callToken, int expected, int actual) {
        return new ParseError(getIllegalNumberOfArgumentsMessage(callToken,expected, actual), callToken.getLine(), callToken.getCharPositionInLine());
    }

    private static String getIllegalNumberOfArgumentsMessage(Token callToken, int expected, int actual) {
        return String.format(ILLEGAL_NUMBER_OF_ARGUMENTS_MESSAGE, callToken.getText(), callToken.getLine(), callToken.getCharPositionInLine(), expected, actual);
    }

    public static ParseError unknownSymbol(Token token) {
        return new ParseError(getUnknownSymbolMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getUnknownSymbolMessage(Token token) {
        return String.format(UNKNOWN_SYMBOL_MESSAGE,token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError illegalUseOfName(Token token) {
        return new ParseError(getIllegalUseOfNameMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getIllegalUseOfNameMessage(Token token) {
        return String.format(ILLEGAL_USE_OF_NAME_MESSAGE, token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError illegalUseOfArraySyntax(Token token) {
        return new ParseError(getIllegalUseOfArraySyntaxMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getIllegalUseOfArraySyntaxMessage(Token token) {
        return String.format(ILLEGAL_USE_OF_ARRAY_SYNTAX, token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError unknownState(Token token) {
        return new ParseError(getUnknownStateMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getUnknownStateMessage(Token token) {
        return String.format(UNKNOWN_STATE_MESSAGE, token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError unknownVariable(Token token) {
        return new ParseError(getUnknownVariableMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getUnknownVariableMessage(Token token) {
        return String.format(UNKNOWN_VARIABLE_MESSAGE, token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError illegalRangeInterval(Token token) {
        return new ParseError(getIllegalRangeIntervalMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getIllegalRangeIntervalMessage(Token token) {
        return String.format(ILLEGAL_RANGE_INTERVAL_MESSAGE, token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError rangeIntervalIsMissing(Token token) {
        return new ParseError(getRangeIntervalIsMissingMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getRangeIntervalIsMissingMessage(Token token) {
        return String.format(RANGE_INTERVAL_IS_MISSING_MESSAGE, token.getText(), token.getLine(), token.getCharPositionInLine());
    }

    public static ParseError unknownType(Token name) {
        return new ParseError(getUnknownTypeMessage(name), name.getLine(), name.getCharPositionInLine());
    }

    private static String getUnknownTypeMessage(Token name) {
        return String.format(UNKNOWN_TYPE_MESSAGE, name.getText(), name.getLine(), name.getCharPositionInLine());
    }

    public static ParseError illegalUseOfRandomExpression(Token token) {
        return new ParseError(getIllegalUseOfRandomExpressionMessage(token), token.getLine(), token.getCharPositionInLine());
    }

    private static String getIllegalUseOfRandomExpressionMessage(Token token) {
        return String.format(ILLEGAL_USE_OF_RANDOM_EXPRESSION, token.getLine(), token.getCharPositionInLine());
    }
}
