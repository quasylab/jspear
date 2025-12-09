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

import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.semantics.*;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;

public class JSpearEnvironmentGenerator extends JSpearSpecificationLanguageBaseVisitor<JSpearEnvironmentUpdateFunction> {

    private final JSpearVariableAllocation allocation;
    private final JSpearExpressionEvaluationContext context;
    private final JSpearVariableRegistry registry;

    public JSpearEnvironmentGenerator(JSpearVariableAllocation allocation, JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry) {
        this.allocation = allocation;
        this.context = context;
        this.registry = registry;
    }

    @Override
    public JSpearEnvironmentUpdateFunction visitEnvironmentBlock(JSpearSpecificationLanguageParser.EnvironmentBlockContext ctx) {
        return new JSpearEnvironmentBlockFunction(this.allocation, ctx.commands.stream().map(c -> c.accept(this)).toList());
    }

    @Override
    public JSpearEnvironmentUpdateFunction visitEnvironmentAssignment(JSpearSpecificationLanguageParser.EnvironmentAssignmentContext ctx) {
        return new JSpearEnvironmentAssignmentFunction(this.allocation, getEnvironmentAssignmentFunction(ctx.variableAssignment()));
    }

    private BiFunction<RandomGenerator, JSpearStore, Optional<DataStateUpdate>> getEnvironmentAssignmentFunction(JSpearSpecificationLanguageParser.VariableAssignmentContext variableAssignmentContext) {
        JSpearVariable variable = registry.get(JSpearVariable.getTargetVariableName(variableAssignmentContext.target.name.getText()));
        JSpearExpressionEvaluationFunction valueFunction = JSpearExpressionEvaluator.eval(context, registry, variableAssignmentContext.value);
        if (variableAssignmentContext.guard != null) {
            JSpearExpressionEvaluationFunction guardFunction = JSpearExpressionEvaluator.eval(context, registry, variableAssignmentContext.guard);
            return (rg, s) -> (JSpearValue.isTrue(guardFunction.eval(rg, s))?allocation.set(variable, valueFunction.eval(rg, s)):Optional.empty());
        } else {
            return (rg, s) -> allocation.set(variable, valueFunction.eval(rg, s));
        }
    }


    @Override
    public JSpearEnvironmentUpdateFunction visitEnvironmentIfThenElse(JSpearSpecificationLanguageParser.EnvironmentIfThenElseContext ctx) {
        if (ctx.elseCommand != null) {
            return new JSpearEnvironmentConditionalUpdateFunction(
                    this.allocation,
                    JSpearExpressionEvaluator.eval(context, registry, ctx.guard),
                    ctx.thenCommand.accept(this),
                    ctx.elseCommand.accept(this));
        } else {
            return new JSpearEnvironmentConditionalUpdateFunction(
                    this.allocation,
                    JSpearExpressionEvaluator.eval(context, registry, ctx.guard),
                    ctx.thenCommand.accept(this));

        }
    }

    @Override
    public JSpearEnvironmentUpdateFunction visitEnvironmentLetCommand(JSpearSpecificationLanguageParser.EnvironmentLetCommandContext ctx) {
        JSpearVariable[] variables = new JSpearVariable[ctx.localVariables.size()];
        JSpearExpressionEvaluationFunction[] localVariablesValues = new JSpearExpressionEvaluationFunction[variables.length];
        for(int i=0; i<variables.length; i++) {
            variables[i] = registry.getOrRegister(ctx.localVariables.get(i).name.getText());
            localVariablesValues[i] = JSpearExpressionEvaluator.eval(context, registry, ctx.localVariables.get(i).expression());
        }
        return new JSpearEnvironmentLetUpdateFunction(allocation, variables, localVariablesValues, ctx.body.accept(this));
    }
}
