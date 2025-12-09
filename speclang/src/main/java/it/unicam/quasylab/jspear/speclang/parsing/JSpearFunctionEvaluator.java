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

import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluationFunction;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.JSpearExpressionEvaluationContext;
import it.unicam.quasylab.jspear.speclang.variables.JSpearStore;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariable;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableRegistry;

public class JSpearFunctionEvaluator extends JSpearSpecificationLanguageBaseVisitor<JSpearExpressionEvaluationFunction> {

    private final JSpearExpressionEvaluationContext context;

    private final JSpearVariableRegistry registry;

    public JSpearFunctionEvaluator(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry) {
        this.context = context;
        this.registry = registry;
    }

    public static JSpearExpressionEvaluationFunction eval(JSpearExpressionEvaluationContext context, JSpearVariableRegistry registry, JSpearSpecificationLanguageParser.FunctionBlockStatementContext functionBlockStatement) {
        return functionBlockStatement.accept(new JSpearFunctionEvaluator(context, registry));
    }


    @Override
    public JSpearExpressionEvaluationFunction visitFunctionLetStatement(JSpearSpecificationLanguageParser.FunctionLetStatementContext ctx) {
        JSpearExpressionEvaluationFunction valueFunction = JSpearExpressionEvaluator.eval(context, registry, ctx.value);
        JSpearVariable variable = registry.getOrRegister(ctx.name.getText());
        JSpearExpressionEvaluationFunction bodyFunction = ctx.body.accept(this);
        return (rg, s) -> bodyFunction.eval(rg, JSpearStore.let(variable, valueFunction.eval(rg, s), s));
    }

    @Override
    public JSpearExpressionEvaluationFunction visitFunctionIfThenElseStatement(JSpearSpecificationLanguageParser.FunctionIfThenElseStatementContext ctx) {
        JSpearExpressionEvaluationFunction guardFunction = JSpearExpressionEvaluator.eval(context, registry, ctx.guard);
        JSpearExpressionEvaluationFunction thenFunction = ctx.thenStatement.accept(this);
        JSpearExpressionEvaluationFunction elseFunction = ctx.elseStatement.accept(this);
        return (rg, s) -> {
            if (JSpearValue.isTrue(guardFunction.eval(rg, s))) {
                return thenFunction.eval(rg, s);
            } else {
                return elseFunction.eval(rg, s);
            }
        };
    }

    @Override
    public JSpearExpressionEvaluationFunction visitFunctionReturnStatement(JSpearSpecificationLanguageParser.FunctionReturnStatementContext ctx) {
        return JSpearExpressionEvaluator.eval(context, registry, ctx.expression());
    }

    @Override
    public JSpearExpressionEvaluationFunction visitFunctionBlockStatement(JSpearSpecificationLanguageParser.FunctionBlockStatementContext ctx) {
        return ctx.functionStatement().accept(this);
    }
}
