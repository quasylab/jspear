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

import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.perturbation.SequentialPerturbation;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluationFunction;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;

public class JSpearPerturbationGenerator extends JSpearSpecificationLanguageBaseVisitor<Perturbation> {
    private final JSpearVariableAllocation allocation;
    private final JSpearExpressionEvaluationContext context;
    private final JSpearVariableRegistry registry;
    private final Map<String, Perturbation> perturbationMap;

    public JSpearPerturbationGenerator(JSpearVariableAllocation allocation, JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, Map<String, Perturbation> perturbationMap) {
        this.allocation = allocation;
        this.context = context;
        this.registry = registry;
        this.perturbationMap = perturbationMap;
    }

    @Override
    public Perturbation visitPerturbationExpressionReference(JSpearSpecificationLanguageParser.PerturbationExpressionReferenceContext ctx) {
        return perturbationMap.getOrDefault(ctx.name.getText(), Perturbation.NONE);
    }

    @Override
    public Perturbation visitPerturbationExpressionBrackets(JSpearSpecificationLanguageParser.PerturbationExpressionBracketsContext ctx) {
        return ctx.perturbationExpression().accept(this);
    }

    @Override
    public Perturbation visitPerturbationExpressionIteration(JSpearSpecificationLanguageParser.PerturbationExpressionIterationContext ctx) {
        int iteration = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.iterationValue));
        return new IterativePerturbation(iteration, ctx.argument.accept(this));
    }

    @Override
    public Perturbation visitPerturbationExpressionAtomic(JSpearSpecificationLanguageParser.PerturbationExpressionAtomicContext ctx) {
        int iteration = JSpearValue.intValue(JSpearExpressionEvaluator.evalToValue(context, registry, ctx.time));
        return new AtomicPerturbation(iteration, getAssignment(ctx.assignments));
    }

    private DataStateFunction getAssignment(List<JSpearSpecificationLanguageParser.PerturbationAssignmentContext> assignments) {
        List<BiFunction<RandomGenerator, JSpearStore, DataStateUpdate>> updates = assignments.stream().map(this::getAssignment).toList();
        return (rg, ds) -> {
            JSpearStore store = JSpearStore.storeOf(allocation,ds);
            return ds.apply(updates.stream().map(uf -> uf.apply(rg, store)).toList());
        };
    }

    private BiFunction<RandomGenerator, JSpearStore, DataStateUpdate> getAssignment(JSpearSpecificationLanguageParser.PerturbationAssignmentContext assignment) {
        JSpearVariable variable = registry.get(assignment.name.getText());
        JSpearExpressionEvaluationFunction value = JSpearExpressionEvaluator.eval(context, registry, assignment.value);
        return (rg, s) -> allocation.set(variable, value.eval(rg, s)).get();
    }

        @Override
    public Perturbation visitPerturbationExpressionSequence(JSpearSpecificationLanguageParser.PerturbationExpressionSequenceContext ctx) {
        return new SequentialPerturbation(ctx.first.accept(this), ctx.second.accept(this));
    }

    @Override
    public Perturbation visitPerturbationExpressionNil(JSpearSpecificationLanguageParser.PerturbationExpressionNilContext ctx) {
        return Perturbation.NONE;
    }
}
