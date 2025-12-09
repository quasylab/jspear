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

package it.unicam.quasylab.jspear.speclang.semantics;

import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.variables.*;
import it.unicam.quasylab.jspear.speclang.values.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.DoubleUnaryOperator;

public class JSpearExpressionEvaluator extends JSpearSpecificationLanguageBaseVisitor<JSpearExpressionEvaluationFunction> {


    private final static Map<String, BiFunction<JSpearValue, JSpearValue, JSpearValue>> binaryOperators = Map.of(
            "+", JSpearValue::sum,
            "*", JSpearValue::product,
            "-", JSpearValue::subtraction,
            "/", JSpearValue::division, //TODO: Check how to handle division by zero!
            "%", JSpearValue::modulo,
            "atan2", (x,y) -> JSpearValue.apply(Math::atan2, x, y),
            "hypot", (x,y) -> JSpearValue.apply(Math::hypot, x, y),
            "max", (x,y) -> JSpearValue.apply(Math::max, x, y),
            "min", (x,y) -> JSpearValue.apply(Math::min, x, y),
            "pow",   (x,y) -> JSpearValue.apply(Math::pow, x, y)
    );

    private final static Map<String, DoubleUnaryOperator> unaryOperators = Map.ofEntries(
            Map.entry("+", x -> +x),
            Map.entry("-", x -> -x),
            Map.entry("abs", Math::abs),
            Map.entry("acos", Math::acos),
            Map.entry("asin", Math::asin),
            Map.entry("atan", Math::atan),
            Map.entry("cbrt", Math::cbrt),
            Map.entry("ceil", Math::ceil),
            Map.entry("cos", Math::cos),
            Map.entry("cosh", Math::cosh),
            Map.entry("exp", Math::exp),
            Map.entry("expm1", Math::expm1),
            Map.entry("floor", Math::floor),
            Map.entry("log", Math::log),
            Map.entry("log10", Math::log10),
            Map.entry("log1p", Math::log1p),
            Map.entry("signum", Math::signum),
            Map.entry("sin", Math::sin),
            Map.entry("sinh", Math::sinh),
            Map.entry("sqrt", Math::sqrt),
            Map.entry("tan", Math::tan)
    );

    private final JSpearExpressionEvaluationContext context;

    private final JSpearVariableRegistry registry;


    public JSpearExpressionEvaluator(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry) {
        this.context = context;
        this.registry = registry;
    }

    public static JSpearValue evalToValue(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, JSpearSpecificationLanguageParser.ExpressionContext expression) {
        return expression.accept(new JSpearExpressionEvaluator(context, registry)).eval(null, null);
    }

    public static JSpearExpressionEvaluationFunction eval(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, JSpearSpecificationLanguageParser.ExpressionContext expression) {
        return expression.accept(new JSpearExpressionEvaluator(context, registry));
    }

    public static DataStateExpression evalToDataStateExpression(JSpearVariableAllocation allocation, JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, JSpearSpecificationLanguageParser.ExpressionContext expression) {
        JSpearExpressionEvaluationFunction evaluation = expression.accept(new JSpearExpressionEvaluator(context, registry));
        return ds -> evaluation.eval(JSpearStore.storeOf(allocation, ds)).toDouble();
    }

    @Override
    public JSpearExpressionEvaluationFunction visitNegationExpression(JSpearSpecificationLanguageParser.NegationExpressionContext ctx) {
        JSpearExpressionEvaluationFunction arg = ctx.arg.accept(this);
        return (rg, s) -> JSpearValue.negate(arg.eval(rg, s));
    }

    @Override
    public JSpearExpressionEvaluationFunction visitExponentExpression(JSpearSpecificationLanguageParser.ExponentExpressionContext ctx) {
        JSpearExpressionEvaluationFunction leftEvaluation = ctx.left.accept(this);
        JSpearExpressionEvaluationFunction rightEvaluation = ctx.right.accept(this);
        return (rg, s) -> JSpearValue.apply(Math::pow, leftEvaluation.eval(rg, s), rightEvaluation.eval(rg, s));
    }

    private JSpearExpressionEvaluationFunction evalBinary(BiFunction<JSpearValue, JSpearValue, JSpearValue> op, JSpearSpecificationLanguageParser.ExpressionContext firstArgument, JSpearSpecificationLanguageParser.ExpressionContext secondArgument) {
        JSpearExpressionEvaluationFunction firstArgumentEvaluation = firstArgument.accept(this);
        JSpearExpressionEvaluationFunction secondArgumentEvaluation = secondArgument.accept(this);
        return (rg, s) -> op.apply(firstArgumentEvaluation.eval(rg, s), secondArgumentEvaluation.eval(rg, s));
    }


    @Override
    public JSpearExpressionEvaluationFunction visitBinaryMathCallExpression(JSpearSpecificationLanguageParser.BinaryMathCallExpressionContext ctx) {
        return evalBinary(getBinaryOperator(ctx.binaryMathFunction().start.getText()), ctx.left, ctx.right);
    }

    private BiFunction<JSpearValue, JSpearValue, JSpearValue> getBinaryOperator(String op) {
        return binaryOperators.getOrDefault(op, (x,y) -> JSpearValue.ERROR_VALUE);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitTrueValue(JSpearSpecificationLanguageParser.TrueValueContext ctx) {
        return JSpearExpressionEvaluationFunction.of( JSpearBoolean.TRUE );
    }


    @Override
    public JSpearExpressionEvaluationFunction visitRelationExpression(JSpearSpecificationLanguageParser.RelationExpressionContext ctx) {
        return evalBinary(getRelationOperator(ctx.op.getText()),ctx.left, ctx.right);
    }

    private BiFunction<JSpearValue, JSpearValue, JSpearValue> getRelationOperator(String op) {
        return switch (op) {
            case "<" -> JSpearValue::isLessThan;
            case "<=" -> JSpearValue::isLessOrEqualThan;
            case "==" -> JSpearValue::isEqualTo;
            case ">=" -> JSpearValue::isGreaterOrEqualThan;
            case ">" -> JSpearValue::isGreaterThan;
            default -> (x, y) -> JSpearValue.ERROR_VALUE;
        };
    }

    @Override
    public JSpearExpressionEvaluationFunction visitBracketExpression(JSpearSpecificationLanguageParser.BracketExpressionContext ctx) {
        return ctx.expression().accept(this);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitFalseValue(JSpearSpecificationLanguageParser.FalseValueContext ctx) {
        return JSpearExpressionEvaluationFunction.of(JSpearBoolean.FALSE);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitAndExpression(JSpearSpecificationLanguageParser.AndExpressionContext ctx) {
        return evalBinary(JSpearValue::and, ctx.left, ctx.right);
    }


    @Override
    public JSpearExpressionEvaluationFunction visitUnaryMathCallExpression(JSpearSpecificationLanguageParser.UnaryMathCallExpressionContext ctx) {
        return evalUnary(unaryOperators.get(ctx.unaryMathFunction().start.getText()), ctx.argument);
    }

    private JSpearExpressionEvaluationFunction evalUnary(DoubleUnaryOperator op, JSpearSpecificationLanguageParser.ExpressionContext argument) {
        JSpearExpressionEvaluationFunction argumentEvaluator = argument.accept(this);
        return (rg, s) -> JSpearValue.apply(op, argumentEvaluator.eval(rg, s));
    }


    @Override
    public JSpearExpressionEvaluationFunction visitUnaryExpression(JSpearSpecificationLanguageParser.UnaryExpressionContext ctx) {
        return evalUnary(unaryOperators.get(ctx.op.getText()), ctx.arg);
    }


    @Override
    public JSpearExpressionEvaluationFunction visitReferenceExpression(JSpearSpecificationLanguageParser.ReferenceExpressionContext ctx) {
        String name = ctx.name.getText();
        return  getTargetEvaluationFunction(name);
    }

    private JSpearExpressionEvaluationFunction getTargetEvaluationFunction(String name) {
        if (context.isDefined(name)) {
            return JSpearExpressionEvaluationFunction.of(context.get(name));
        }
        if (registry.isDeclared(name)) {
            JSpearVariable variable = registry.get(name);
            return (rg, s) -> s.get(variable);
        }
        return JSpearExpressionEvaluationFunction.of(JSpearValue.ERROR_VALUE);
    }


    @Override
    public JSpearExpressionEvaluationFunction visitIntValue(JSpearSpecificationLanguageParser.IntValueContext ctx) {
        JSpearValue value = new JSPearInteger(Integer.parseInt(ctx.getText()));
        return JSpearExpressionEvaluationFunction.of(value);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitNormalExpression(JSpearSpecificationLanguageParser.NormalExpressionContext ctx) {
        JSpearExpressionEvaluationFunction meanEvaluation = ctx.mean.accept(this);
        JSpearExpressionEvaluationFunction varianceEvaluation = ctx.variance.accept(this);
        return (rg, s) -> JSpearValue.sampleNormal(rg, meanEvaluation.eval(rg, s), varianceEvaluation.eval(rg, s));
    }

    @Override
    public JSpearExpressionEvaluationFunction visitUniformExpression(JSpearSpecificationLanguageParser.UniformExpressionContext ctx) {
        JSpearExpressionEvaluationFunction[] elements = ctx.expression().stream().map(e -> e.accept(this)).toArray(JSpearExpressionEvaluationFunction[]::new);
        return (rg, s) -> {
            int selected = rg.nextInt(elements.length);
            return elements[selected].eval(rg, s);
        };
    }


    @Override
    public JSpearExpressionEvaluationFunction visitOrExpression(JSpearSpecificationLanguageParser.OrExpressionContext ctx) {
        JSpearExpressionEvaluationFunction leftEvaluation = ctx.left.accept(this);
        JSpearExpressionEvaluationFunction rightEvaluation = ctx.right.accept(this);
        return (rg, s) -> JSpearValue.or( leftEvaluation.eval(rg, s), rightEvaluation.eval(rg, s));
    }

    @Override
    public JSpearExpressionEvaluationFunction visitIfThenElseExpression(JSpearSpecificationLanguageParser.IfThenElseExpressionContext ctx) {
        JSpearExpressionEvaluationFunction guardEvaluation = ctx.guard.accept(this);
        JSpearExpressionEvaluationFunction thenEvaluation = ctx.thenBranch.accept(this);
        JSpearExpressionEvaluationFunction elseEvaluation = ctx.elseBranch.accept(this);
        return (rg, s) -> JSpearValue.ifThenElse(guardEvaluation.eval(rg, s), () -> thenEvaluation.eval(rg, s), () -> elseEvaluation.eval(rg, s));
    }

    @Override
    public JSpearExpressionEvaluationFunction visitRealValue(JSpearSpecificationLanguageParser.RealValueContext ctx) {
        JSpearValue value = new JSpearReal(Double.parseDouble(ctx.getText()));
        return JSpearExpressionEvaluationFunction.of(value);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitCallExpression(JSpearSpecificationLanguageParser.CallExpressionContext ctx) {
        String functionName = ctx.name.getText();
        if (context.isAFunction(functionName)) {
            JSpearExpressionEvaluationFunction[] arguments = ctx.callArguments.stream().map(e -> e.accept(this)).toArray(JSpearExpressionEvaluationFunction[]::new);
            JSpearFunction function = context.getFunction(functionName);
            return (rg, s) -> function.apply(rg, Arrays.stream(arguments).map(e -> e.eval(rg, s)).toArray(JSpearValue[]::new) );
        } else {
            return JSpearExpressionEvaluationFunction.of(JSpearValue.ERROR_VALUE);
        }
    }



    @Override
    public JSpearExpressionEvaluationFunction visitMulDivExpression(JSpearSpecificationLanguageParser.MulDivExpressionContext ctx) {
        return evalBinary(getBinaryOperator(ctx.op.getText()), ctx.left, ctx.right);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitAddSubExpression(JSpearSpecificationLanguageParser.AddSubExpressionContext ctx) {
        return evalBinary(getBinaryOperator(ctx.op.getText()), ctx.left, ctx.right);
    }

    @Override
    public JSpearExpressionEvaluationFunction visitRandomExpression(JSpearSpecificationLanguageParser.RandomExpressionContext ctx) {
        if (ctx.from == null) {
            return (rg, s) -> new JSpearReal(rg.nextDouble());
        } else {
            JSpearExpressionEvaluationFunction fromEvaluation = ctx.from.accept(this);
            JSpearExpressionEvaluationFunction toEvaluation = ctx.to.accept(this);
            return (rg, s) -> JSpearValue.sample(rg, fromEvaluation.eval(rg, s), toEvaluation.eval(rg, s));
        }
    }



    private JSpearExpressionEvaluationFunction getExpressionEvaluationFunction(JSpearSpecificationLanguageParser.ExpressionContext expression) {
        return expression.accept(new JSpearExpressionEvaluator(context, registry));
    }


}
