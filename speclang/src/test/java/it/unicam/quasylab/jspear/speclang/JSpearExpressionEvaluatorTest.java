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

package it.unicam.quasylab.jspear.speclang;

import it.unicam.quasylab.jspear.speclang.parsing.ParseErrorCollector;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.types.ExpressionTypeInference;
import it.unicam.quasylab.jspear.speclang.types.JSpearRandomType;
import it.unicam.quasylab.jspear.speclang.types.JSpearType;
import it.unicam.quasylab.jspear.speclang.types.LocalTypeContext;
import it.unicam.quasylab.jspear.speclang.values.*;
import it.unicam.quasylab.jspear.speclang.variables.JSpearExpressionEvaluationContext;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

class JSpearExpressionEvaluatorTest {

    private  static Map<String, JSpearValue> valuesTests = new HashMap<>();


    @BeforeAll
    public static void initTypeTests() {
        valuesTests.put("2", new JSPearInteger(2));
        valuesTests.put("2.", new JSpearReal(2.0));
        valuesTests.put("true", JSpearBoolean.TRUE);
        valuesTests.put("false", JSpearBoolean.FALSE);
        valuesTests.put("2+3", new JSPearInteger(5));
        valuesTests.put("2.+3", new JSpearReal(5.0));
        valuesTests.put("2+3.", new JSpearReal(5.0));
        valuesTests.put("2.+3.", new JSpearReal(5.0));
        valuesTests.put("true & true", JSpearBoolean.TRUE);
        valuesTests.put("true | true", JSpearBoolean.TRUE);
        valuesTests.put("2 ^ 3", new JSpearReal(Math.pow(2, 3)));
        valuesTests.put("2 * 3", new JSPearInteger(6));
        valuesTests.put("2. * 3", new JSpearReal(6.0));
        valuesTests.put("2 * 3.", new JSpearReal(6.0));
        valuesTests.put("2. * 3.", new JSpearReal(6.0));
        valuesTests.put("2 < 3", JSpearBoolean.TRUE);
        valuesTests.put("2. <= 3", JSpearBoolean.TRUE);
        valuesTests.put("2 == 3.", JSpearBoolean.FALSE);
        valuesTests.put("2. > 3.", JSpearBoolean.FALSE);
        valuesTests.put("2. >= 3", JSpearBoolean.FALSE);
        valuesTests.put("!true", JSpearBoolean.FALSE);
        valuesTests.put("(2<3?1.0:2.0)", new JSpearReal(1.0));
        valuesTests.put("(2>3?1.0:2)", new JSPearInteger(2));
        valuesTests.put("abs(1)", new JSpearReal(Math.abs(1)));
        valuesTests.put("acos(1)", new JSpearReal(Math.acos(1)));
        valuesTests.put("asin(1)", new JSpearReal(Math.asin(1)));
        valuesTests.put("atan(1)", new JSpearReal(Math.atan(1)));
        valuesTests.put("cbrt(1)", new JSpearReal(Math.cbrt(1)));
        valuesTests.put("ceil(1)", new JSpearReal(Math.ceil(1)));
        valuesTests.put("cos(1)", new JSpearReal(Math.cos(1)));
        valuesTests.put("cosh(1)", new JSpearReal(Math.cosh(1)));
        valuesTests.put("exp(1)", new JSpearReal(Math.exp(1)));
        valuesTests.put("expm1(1)", new JSpearReal(Math.expm1(1)));
        valuesTests.put("floor(1)", new JSpearReal(Math.floor(1)));
        valuesTests.put("log(2)", new JSpearReal(Math.log(2)));
        valuesTests.put("log10(2)", new JSpearReal(Math.log10(2)));
        valuesTests.put("log1p(2)", new JSpearReal(Math.log1p(2)));
        valuesTests.put("signum(1)", new JSpearReal(Math.signum(1)));
        valuesTests.put("sin(1)", new JSpearReal(Math.sin(1)));
        valuesTests.put("sinh(1)", new JSpearReal(Math.sinh(1)));
        valuesTests.put("sqrt(1)", new JSpearReal(Math.sqrt(1)));
        valuesTests.put("tan(1)", new JSpearReal(Math.tan(1)));
        valuesTests.put("atan2(1,2)", new JSpearReal(Math.atan2(1,2)));
        valuesTests.put("hypot(1,2)", new JSpearReal(Math.hypot(1,2)));
        valuesTests.put("max(1,2)", new JSpearReal(Math.max(1,2)));
        valuesTests.put("min(1,2)", new JSpearReal(Math.min(1,2)));
        valuesTests.put("pow(2,3)", new JSpearReal(Math.pow(2,3)));

    }

    private ParseTree getParseTree(String code) {
        JSpearSpecificationLanguageLexer lexer = new JSpearSpecificationLanguageLexer(CharStreams.fromString(code));
        CommonTokenStream tokens =  new CommonTokenStream(lexer);
        JSpearSpecificationLanguageParser parser = new JSpearSpecificationLanguageParser(tokens);
        return parser.expression();
    }


    @Test
    void shouldInferIntegerType() {
        ParseTree parseTree = getParseTree("2");
        assertEquals(JSpearType.INTEGER_TYPE, inferTypeOf(parseTree));
    }

    @Test
    void shouldInferIntegerTypeFromVariable() {
        ParseTree parseTree = getParseTree("x");
        assertEquals(JSpearType.INTEGER_TYPE, inferTypeOf(Map.of("x", JSpearType.INTEGER_TYPE), parseTree));
    }

    @Test
    void shouldInferRealTypeFromVariable() {
        ParseTree parseTree = getParseTree("x");
        assertEquals(JSpearType.REAL_TYPE, inferTypeOf(Map.of("x", JSpearType.REAL_TYPE), parseTree));
    }

    @Test
    void shouldInferBooleanTypeFromVariable() {
        ParseTree parseTree = getParseTree("x");
        assertEquals(JSpearType.BOOLEAN_TYPE, inferTypeOf(Map.of("x", JSpearType.BOOLEAN_TYPE), parseTree));
    }


    @Test
    void shouldInferRealType() {
        ParseTree parseTree = getParseTree("2.");
        assertEquals(JSpearType.REAL_TYPE, inferTypeOf(parseTree));
    }


    @Test
    void shouldInferBooleanTypeFromTrue() {
        ParseTree parseTree = getParseTree("true");
        assertEquals(JSpearType.BOOLEAN_TYPE, inferTypeOf(parseTree));
    }

    @Test
    void shouldInferBooleanTypeFromFalse() {
        ParseTree parseTree = getParseTree("false");
        assertEquals(JSpearType.BOOLEAN_TYPE, inferTypeOf(parseTree));
    }

    @Test
    void shouldInferRandomBooleanType() {
        ParseTree parseTree = getParseTree("R < R");
        assertEquals(new JSpearRandomType(JSpearType.BOOLEAN_TYPE), inferTypeOf(true, parseTree));
    }

    @Test
    void shouldInferRandomRealType() {
        ParseTree parseTree = getParseTree("R");
        assertEquals(new JSpearRandomType(JSpearType.REAL_TYPE), inferTypeOf(true, parseTree));
    }


    private JSpearType inferTypeOf(ParseTree parseTree) {
        return inferTypeOf(false, parseTree);
    }

    private JSpearType inferTypeOf(boolean randomExpressionAllowed, ParseTree parseTree) {
        return inferTypeOf(Map.of(), randomExpressionAllowed, parseTree);
    }

    private JSpearType inferTypeOf(Map<String, JSpearType> types, ParseTree expression) {
        return inferTypeOf(types, false, expression);
    }

    private JSpearType inferTypeOf(Map<String, JSpearType> types, boolean randomExpressionAllowed, ParseTree expression) {
        ExpressionTypeInference inference = new ExpressionTypeInference(new LocalTypeContext(types), new ParseErrorCollector(), randomExpressionAllowed);
        return expression.accept(inference);
    }


    private JSpearValue evalExpression(ParseTree expression, Map<String, JSpearValue> args) {
        return expression.accept(new JSpearExpressionEvaluator(new JSpearExpressionEvaluationContext(args), null)).eval();
    }

    @Test
    public void testExpressions() {
        for (Map.Entry<String, JSpearValue> test: valuesTests.entrySet()) {
            assertEquals(test.getValue(), evalExpression(getParseTree(test.getKey()), new HashMap<>()), test.getKey());
        }
    }


}