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

import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.*;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.JSpearExpressionEvaluationContext;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableRegistry;

import java.util.Map;

public class JSpearRobustnessFormulaGenerator extends JSpearSpecificationLanguageBaseVisitor<RobustnessFormula> {
    private final JSpearVariableAllocation allocation;
    private final JSpearExpressionEvaluationContext context;
    private final JSpearVariableRegistry registry;
    private final Map<String, DistanceExpression> distanceMap;
    private final Map<String, Perturbation> perturbationMap;
    private final Map<String, RobustnessFormula> formulaMap;

    public JSpearRobustnessFormulaGenerator(JSpearVariableAllocation allocation, JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, Map<String, Perturbation> perturbationMap, Map<String, DistanceExpression> distanceExpressionMap, Map<String, RobustnessFormula> formulaMap) {
        this.allocation = allocation;
        this.context = context;
        this.registry = registry;
        this.distanceMap = distanceExpressionMap;
        this.perturbationMap = perturbationMap;
        this.formulaMap = formulaMap;
    }

    @Override
    public RobustnessFormula visitRobtlFormulaDistance(JSpearSpecificationLanguageParser.RobtlFormulaDistanceContext ctx) {
        double value = JSpearExpressionEvaluator.evalToValue(context, registry, ctx.value).toDouble();
        return new AtomicRobustnessFormula(perturbationMap.get(ctx.perturbationReference.getText()), distanceMap.get(ctx.expressionReference.getText()), RelationOperator.get(ctx.op.getText()), value);
    }

    @Override
    public RobustnessFormula visitRobtlFormulaConjunction(JSpearSpecificationLanguageParser.RobtlFormulaConjunctionContext ctx) {
        return new ConjunctionRobustnessFormula(ctx.left.accept(this), ctx.right.accept(this));
    }

    @Override
    public RobustnessFormula visitRobtlFormulaTrue(JSpearSpecificationLanguageParser.RobtlFormulaTrueContext ctx) {
        return new TrueRobustnessFormula();
    }

    @Override
    public RobustnessFormula visitRobtlFormulaReference(JSpearSpecificationLanguageParser.RobtlFormulaReferenceContext ctx) {
        return formulaMap.get(ctx.name.getText());
    }

    @Override
    public RobustnessFormula visitRobtlFormulaUntil(JSpearSpecificationLanguageParser.RobtlFormulaUntilContext ctx) {
        int from = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.from));
        int to = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.to));
        return new UntilRobustnessFormula(ctx.left.accept(this), from, to, ctx.right.accept(this));
    }

    @Override
    public RobustnessFormula visitRobtlFormulaNegation(JSpearSpecificationLanguageParser.RobtlFormulaNegationContext ctx) {
        return new NegationRobustnessFormula(ctx.argument.accept(this));
    }

    @Override
    public RobustnessFormula visitRobtlFormulaDisjunction(JSpearSpecificationLanguageParser.RobtlFormulaDisjunctionContext ctx) {
        return new DisjunctionRobustnessFormula(ctx.left.accept(this), ctx.right.accept(this));
    }

    @Override
    public RobustnessFormula visitRobtlFormulaFalse(JSpearSpecificationLanguageParser.RobtlFormulaFalseContext ctx) {
        return new FalseRobustnessFormula();
    }

    @Override
    public RobustnessFormula visitRobtlFormulaAlways(JSpearSpecificationLanguageParser.RobtlFormulaAlwaysContext ctx) {
        int from = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.from));
        int to = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.to));
        return new AlwaysRobustnessFormula(ctx.argument.accept(this), from, to);
    }

    @Override
    public RobustnessFormula visitRobtlFormulaEventually(JSpearSpecificationLanguageParser.RobtlFormulaEventuallyContext ctx) {
        int from = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.from));
        int to = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.to));
        return new EventuallyRobustnessFormula(ctx.argument.accept(this), from, to);
    }
}
