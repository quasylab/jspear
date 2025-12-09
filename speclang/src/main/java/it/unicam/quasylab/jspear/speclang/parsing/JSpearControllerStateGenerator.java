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

import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.EffectStep;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.controller.JSpearControllerFunction;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluationFunction;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.stream.Collectors;

public class JSpearControllerStateGenerator extends JSpearSpecificationLanguageBaseVisitor<JSpearControllerFunction> {

    private final JSpearExpressionEvaluationContext context;
    private final JSpearVariableRegistry registry;
    private final JSpearVariableAllocation allocation;
    private final Map<String, JSpearControllerFunction> controllerMap;

    private final ControllerRegistry controllerRegistry;


    public JSpearControllerStateGenerator(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, JSpearVariableAllocation allocation, Map<String, JSpearControllerFunction> controllerMap, ControllerRegistry controllerRegistry) {
        this.context = context;
        this.registry = registry;
        this.allocation = allocation;
        this.controllerMap = controllerMap;
        this.controllerRegistry = controllerRegistry;
    }

    public static JSpearControllerFunction generate(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, JSpearVariableAllocation allocation, Map<String, JSpearControllerFunction> controllerMap, ControllerRegistry controllerRegistry, JSpearSpecificationLanguageParser.ControllerBlockBehaviourContext body) {
        return body.accept(new JSpearControllerStateGenerator(context, registry, allocation, controllerMap, controllerRegistry));
    }


    @Override
    public JSpearControllerFunction visitControllerBlockBehaviour(JSpearSpecificationLanguageParser.ControllerBlockBehaviourContext ctx) {
        return JSpearControllerFunction.sequential(ctx.controllerCommand().stream().map(it -> it.accept(this)).collect(Collectors.toList()));
    }

    @Override
    public JSpearControllerFunction visitControllerSequentialBehaviour(JSpearSpecificationLanguageParser.ControllerSequentialBehaviourContext ctx) {
        BiFunction<RandomGenerator, JSpearStore, List<DataStateUpdate>> assignments = getVariableAssignments(ctx.statements);
        JSpearControllerFunction next = ctx.last.accept(this);
        return (rg, ds) -> next.apply(rg, ds).applyBefore(assignments.apply(rg, ds));
    }

    @Override
    public JSpearControllerFunction visitControllerVariableAssignment(JSpearSpecificationLanguageParser.ControllerVariableAssignmentContext ctx) {
        BiFunction<RandomGenerator, JSpearStore, Optional<DataStateUpdate>> assignment = getVariableAssignment(ctx);
        return (rg, ds) -> {
            Optional<DataStateUpdate> result = assignment.apply(rg, ds);
            return result.<EffectStep<Controller>>map(dataStateUpdate -> new EffectStep<>(List.of(dataStateUpdate))).orElseGet(EffectStep::new);
        };
    }

    private BiFunction<RandomGenerator, JSpearStore, List<DataStateUpdate>> getVariableAssignments(List<JSpearSpecificationLanguageParser.ControllerVariableAssignmentContext> statements) {
        List<BiFunction<RandomGenerator, JSpearStore, Optional<DataStateUpdate>>> assignments = statements.stream().map(this::getVariableAssignment).toList();
        return (rg, store) -> assignments.stream().map(a -> a.apply(rg, store)).filter(Optional::isPresent).map(Optional::get).toList();
    }

    @Override
    public JSpearControllerFunction visitControllerCaseStatment(JSpearSpecificationLanguageParser.ControllerCaseStatmentContext ctx) {
        //TODO: FIXME!
        return super.visitControllerCaseStatment(ctx);
    }

    @Override
    public JSpearControllerFunction visitControllerLetAssignment(JSpearSpecificationLanguageParser.ControllerLetAssignmentContext ctx) {
        JSpearVariable variable = registry.getOrRegister(ctx.name.getText());
        JSpearExpressionEvaluationFunction valueFunction = JSpearExpressionEvaluator.eval(context, registry, ctx.value);
        JSpearControllerFunction letBody = ctx.body.accept(this);
        return (rg, store) -> letBody.apply(rg, JSpearStore.let(variable, valueFunction.eval(rg, store), store));
    }

    public BiFunction<RandomGenerator, JSpearStore, Optional<DataStateUpdate>> getVariableAssignment(JSpearSpecificationLanguageParser.ControllerVariableAssignmentContext ctx) {
        JSpearExpressionEvaluationFunction valueFunction = JSpearExpressionEvaluator.eval(context, registry, ctx.value);
        JSpearVariable variable = registry.get(JSpearVariable.getTargetVariableName(ctx.target.name.getText()));
        if (ctx.guard != null) {
            JSpearExpressionEvaluationFunction guardFunction = JSpearExpressionEvaluator.eval(context, registry, ctx.guard);
            return (rg, s) -> {
                if (JSpearValue.isTrue(guardFunction.eval(rg, s))) {
                    return allocation.set(variable, valueFunction.eval(rg, s));
                } else {
                    return Optional.empty();
                }
            };
        } else {
            return (rg, s) -> allocation.set(variable, valueFunction.eval(rg, s));
        }
    }

    @Override
    public JSpearControllerFunction visitControllerExecAction(JSpearSpecificationLanguageParser.ControllerExecActionContext ctx) {
        String referencedName = ctx.target.getText();
        return (rg, store) -> {
            JSpearControllerFunction function = controllerMap.get(referencedName);
            if (function != null) {
                return function.apply(rg, store);
            } else {
                return ControllerRegistry.NIL.next(rg, null);
            }
        };
    }

    @Override
    public JSpearControllerFunction visitControllerStepAtion(JSpearSpecificationLanguageParser.ControllerStepAtionContext ctx) {
        String referencedName = ctx.target.getText();
        Controller controller = controllerRegistry.reference(referencedName);
        if (ctx.steps == null) {
            return (rg, store) -> new EffectStep<>(List.of(), controller);
        } else {
            JSpearExpressionEvaluationFunction steps = JSpearExpressionEvaluator.eval(context, registry, ctx.steps);
            return (rg, store) -> {
                int k = (int) steps.eval(rg, store).toDouble();
                if (k<1) {
                    return new EffectStep<>(List.of(), controller);
                } else {
                    return new EffectStep<>(List.of(), Controller.doTick(k-1, controller));
                }
            };
        }
    }

    @Override
    public JSpearControllerFunction visitControllerIfThenElseBehaviour(JSpearSpecificationLanguageParser.ControllerIfThenElseBehaviourContext ctx) {
        return JSpearControllerFunction.ifThenElse(JSpearExpressionEvaluator.eval(context, registry, ctx.guard),
                ctx.thenBranch.accept(this),
                ctx.elseBranch.accept(this));
    }
}
