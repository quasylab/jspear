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

package it.unicam.quasylab.jspear.speclang.types;

import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.parsing.ParseErrorCollector;
import it.unicam.quasylab.jspear.speclang.parsing.ParseUtil;

public class JSpearFunctionStatementTypeInference extends JSpearSpecificationLanguageBaseVisitor<JSpearType> {
    private final ParseErrorCollector errors;

    private final TypeEvaluationContext context;

    public JSpearFunctionStatementTypeInference(TypeEvaluationContext context, ParseErrorCollector errors) {
        this.errors = errors;
        this.context = context;
    }

    @Override
    public JSpearType visitFunctionLetStatement(JSpearSpecificationLanguageParser.FunctionLetStatementContext ctx) {
        JSpearType letType = ExpressionTypeInference.infer(context, errors, ctx.value);
        if (!letType.isError()) {
            return JSpearFunctionStatementTypeInference.infer(TypeEvaluationContext.letContext(this.context, ctx.name, letType), errors, ctx.body);
        } else {
            return JSpearType.ERROR_TYPE;
        }
    }

    private static JSpearType infer(TypeEvaluationContext context, ParseErrorCollector errors, JSpearSpecificationLanguageParser.FunctionStatementContext ctx) {
        return ctx.accept(new JSpearFunctionStatementTypeInference(context, errors));
    }

    @Override
    public JSpearType visitFunctionIfThenElseStatement(JSpearSpecificationLanguageParser.FunctionIfThenElseStatementContext ctx) {
        JSpearType guardType = ExpressionTypeInference.infer(context, errors, ctx.guard);
        if (!guardType.isBoolean()) {
            errors.record(ParseUtil.typeError(JSpearType.BOOLEAN_TYPE, guardType, ctx.guard.start));
        }
        JSpearType thenType = ctx.thenStatement.accept(this);
        JSpearType elseType = ctx.elseStatement.accept(this);
        if (!thenType.canBeMergedWith(elseType)) {
            errors.record(ParseUtil.typeError(thenType, elseType, ctx.elseStatement.start));
            return JSpearType.ERROR_TYPE;
        }
        JSpearType result = JSpearType.merge(thenType, elseType);
        if (!result.isError()&&guardType.isRandom()) {
            return new JSpearRandomType(result);
        } else {
            return result;
        }
    }

    @Override
    public JSpearType visitFunctionReturnStatement(JSpearSpecificationLanguageParser.FunctionReturnStatementContext ctx) {
        return ExpressionTypeInference.infer(context, errors, ctx.expression());
    }

    @Override
    public JSpearType visitFunctionBlockStatement(JSpearSpecificationLanguageParser.FunctionBlockStatementContext ctx) {
        return ctx.functionStatement().accept(this);
    }
}
