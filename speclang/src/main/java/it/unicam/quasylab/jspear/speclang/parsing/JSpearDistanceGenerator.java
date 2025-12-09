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

import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.distance.UntilDistanceExpression;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.JSpearExpressionEvaluationContext;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableRegistry;

import java.util.Map;

public class JSpearDistanceGenerator extends JSpearSpecificationLanguageBaseVisitor<DistanceExpression> {
    private final JSpearVariableAllocation allocation;
    private final JSpearExpressionEvaluationContext context;
    private final JSpearVariableRegistry registry;
    private final Map<String, DistanceExpression> distanceMap;
    private final Map<String, DataStateExpression> penalties;

    public JSpearDistanceGenerator(JSpearVariableAllocation allocation, JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, Map<String, DistanceExpression> distanceExpressionMap, Map<String, DataStateExpression> penalties) {
        this.allocation = allocation;
        this.context = context;
        this.registry = registry;
        this.distanceMap = distanceExpressionMap;
        this.penalties = penalties;
    }


    @Override
    public DistanceExpression visitDistanceExpressionAtomicLeft(JSpearSpecificationLanguageParser.DistanceExpressionAtomicLeftContext ctx) {
        DataStateExpression expression =  this.penalties.getOrDefault(ctx.panaltyName.getText(), ds -> Double.NaN); //JSpearExpressionEvaluator.evalToDataStateExpression(this.allocation, this.context, this.registry, ctx.expression());
        return new AtomicDistanceExpressionLeq(expression);
    }

    @Override
    public DistanceExpression visitDistanceExpressionAtomicRight(JSpearSpecificationLanguageParser.DistanceExpressionAtomicRightContext ctx) {
        DataStateExpression expression =  this.penalties.getOrDefault(ctx.panaltyName.getText(), ds -> Double.NaN); //JSpearExpressionEvaluator.evalToDataStateExpression(this.allocation, this.context, this.registry, ctx.expression());
        return new AtomicDistanceExpressionGeq(expression);
    }

    @Override
    public DistanceExpression visitDistanceExpressionFinally(JSpearSpecificationLanguageParser.DistanceExpressionFinallyContext ctx) {
        int from = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.from));
        int to = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.to));
        return new MinIntervalDistanceExpression(ctx.argument.accept(this),from, to);
    }

    @Override
    public DistanceExpression visitDistanceExpressionReference(JSpearSpecificationLanguageParser.DistanceExpressionReferenceContext ctx) {
        return distanceMap.get(ctx.name.getText());
    }

    @Override
    public DistanceExpression visitDistanceExpressionMax(JSpearSpecificationLanguageParser.DistanceExpressionMaxContext ctx) {
        return new MaxDistanceExpression(ctx.first.accept(this), ctx.second.accept(this));
    }

    @Override
    public DistanceExpression visitDistanceExpressionUntil(JSpearSpecificationLanguageParser.DistanceExpressionUntilContext ctx) {
        int from = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.from));
        int to = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.to));
        return new UntilDistanceExpression(ctx.left.accept(this), from, to, ctx.right.accept(this));
    }

    @Override
    public DistanceExpression visitDistanceExpressionMin(JSpearSpecificationLanguageParser.DistanceExpressionMinContext ctx) {
        return new MinDistanceExpression(ctx.first.accept(this), ctx.second.accept(this));
    }

    @Override
    public DistanceExpression visitDistanceExpressionGlobally(JSpearSpecificationLanguageParser.DistanceExpressionGloballyContext ctx) {
        int from = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.from));
        int to = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.to));
        return new MaxIntervalDistanceExpression(ctx.argument.accept(this),from, to);
    }

    @Override
    public DistanceExpression visitDistanceExpressionBrackets(JSpearSpecificationLanguageParser.DistanceExpressionBracketsContext ctx) {
        return ctx.distanceExpression().accept(this);
    }

    @Override
    public DistanceExpression visitDistanceExpressionLinearCombination(JSpearSpecificationLanguageParser.DistanceExpressionLinearCombinationContext ctx) {
        double[] weights = ctx.weights.stream().map(exp -> JSpearExpressionEvaluator.evalToValue(context, registry, exp)).mapToDouble(JSpearValue::doubleOf).toArray();
        DistanceExpression[] distanceExpressions = ctx.values.stream().map(de -> de.accept(this)).toArray(DistanceExpression[]::new);
        return new ConvexCombinationDistanceExpression(weights, distanceExpressions);
    }


    @Override
    public DistanceExpression visitDistanceExpressionThreshold(JSpearSpecificationLanguageParser.DistanceExpressionThresholdContext ctx) {
        double value = JSpearExpressionEvaluator.evalToValue(context, registry, ctx.right).toDouble();
        return new ThresholdDistanceExpression(ctx.left.accept(this), RelationOperator.get(ctx.op.getText()), value);
    }
}
