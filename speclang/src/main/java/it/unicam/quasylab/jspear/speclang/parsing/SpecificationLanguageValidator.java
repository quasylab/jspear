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
import it.unicam.quasylab.jspear.speclang.types.*;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;

import java.util.*;
import java.util.stream.Collectors;

public class SpecificationLanguageValidator extends JSpearSpecificationLanguageBaseVisitor<Boolean> {
//    private final ParseErrorCollector errors;
//    private final SymbolTable symbols = new SymbolTable();
//
//    private final Set<String> environmentVariables = new HashSet<>();
//
//    private final GlobalTypeContext globalTypeContext = new GlobalTypeContext();
//
//    public SpecificationLanguageValidator(ParseErrorCollector errors) {
//        this.errors = errors;
//    }
//
//    @Override
//    public Boolean visitJSpearSpecificationModel(JSpearSpecificationLanguageParser.JSpearSpecificationModelContext ctx) {
//        boolean flag = true;
//        for (JSpearSpecificationLanguageParser.ElementContext element: ctx.element()) {
//            flag &= element.accept(this);
//        }
//        return flag;
//    }
//
//    @Override
//    protected Boolean defaultResult() {
//        return true;
//    }
//
//    @Override
//    protected Boolean aggregateResult(Boolean aggregate, Boolean nextResult) {
//        return aggregate&nextResult;
//    }
//
//    @Override
//    public Boolean visitDeclarationFunction(JSpearSpecificationLanguageParser.DeclarationFunctionContext ctx) {
//        if (checkIfNotDuplicated(ctx.name.getText(), ctx)) {
//            JSpearType[] argumentType = ctx.arguments.stream().map(a -> typeOf(a.type())).toArray(JSpearType[]::new);
//            TypeEvaluationContext context = new NestedTypeContext(
//                    new LocalTypeContext(getLocalDeclarations(ctx.arguments)),
//                    this.globalTypeContext);
//            JSpearType returnType = ctx.accept(new JSpearFunctionStatementTypeInference(context, this.errors));
//            if (!returnType.isError()) {
//                this.symbols.recordFunction(ctx.name.getText(), argumentType, returnType, ctx);
//                return true;
//            }
//        }
//        return false;
//    }
//
//
//    private Map<String, JSpearType> getLocalDeclarations(List<JSpearSpecificationLanguageParser.FunctionArgumentContext> arguments) {
//        return arguments.stream().collect(Collectors.toMap(a -> a.name.getText(), a -> typeOf(a.type())));
//    }
//
//    private JSpearType typeOf(JSpearSpecificationLanguageParser.TypeContext type) {
//        if (type instanceof JSpearSpecificationLanguageParser.BooleanTypeContext) {
//            return JSpearType.BOOLEAN_TYPE;
//        }
//        if (type instanceof JSpearSpecificationLanguageParser.IntegerTypeContext) {
//            return JSpearType.INTEGER_TYPE;
//        }
//        if (type instanceof JSpearSpecificationLanguageParser.RealTypeContext) {
//            return JSpearType.REAL_TYPE;
//        }
//        if (type instanceof JSpearSpecificationLanguageParser.CustomTypeContext) {
//            String typeName = ((JSpearSpecificationLanguageParser.CustomTypeContext) type).name.getText();
//            if (!symbols.isACustomType(typeName)) {
//                this.errors.record(ParseUtil.unknownType(((JSpearSpecificationLanguageParser.CustomTypeContext) type).name));
//                return JSpearType.ERROR_TYPE;
//            }
//            return this.symbols.getCustomType(typeName);
//        }
//        return JSpearType.ERROR_TYPE;
//    }
//
//
//    @Override
//    public Boolean visitDeclarationEnvironmnet(JSpearSpecificationLanguageParser.DeclarationEnvironmnetContext ctx) {
//        TypeEvaluationContext localContext = getLocalContext(ctx.localVariables);
//        boolean flag = true;
//        ExpressionTypeInference inference = new ExpressionTypeInference(new NestedTypeContext(localContext, symbols), errors, true);
//        for(JSpearSpecificationLanguageParser.VariableAssignmentContext varAssignment: ctx.assignments) {
//            flag &= checkEnvironmentVariableUpdate(inference, varAssignment);
//        }
//        return flag;
//    }
//
//    private boolean checkEnvironmentVariableUpdate(ExpressionTypeInference inference, JSpearSpecificationLanguageParser.VariableAssignmentContext varAssignment) {
//        boolean flag = (varAssignment.guard == null || inference.checkType(JSpearType.BOOLEAN_TYPE, varAssignment.guard));
//        Optional<JSpearType> expectedType = retrieveAndCheckVariableExpression(inference, varAssignment.target);
//        return flag&expectedType.map(t -> inference.checkType(t, varAssignment.value)).orElse(false);
//    }
//
//    @Override
//    public Boolean visitDeclarationConstant(JSpearSpecificationLanguageParser.DeclarationConstantContext ctx) {
//        if (checkIfNotDuplicated(ctx.name.getText(), ctx)) {
//            JSpearType type = ctx.expression().accept(new ExpressionTypeInference(symbols, errors));
//            if (!type.isError()) {
//                symbols.recordConstant(ctx.name.getText(), type, ctx);
//                return true;
//            } else {
//                return false;
//            }
//        }
//        return false;
//    }
//
//    @Override
//    public Boolean visitDeclarationType(JSpearSpecificationLanguageParser.DeclarationTypeContext ctx) {
//        String customTypeName = ctx.name.getText();
//        if (checkIfNotDuplicated(customTypeName, ctx)
//            &&ctx.elements.stream().allMatch(e -> checkCustomTypeElement(ctx, e))) {
//            symbols.recordCustomType(ctx);
//            return true;
//        }
//        return false;
//    }
//
//    private boolean checkCustomTypeElement(JSpearSpecificationLanguageParser.TypeDeclarationContext ctx, JSpearSpecificationLanguageParser.TypeElementDeclarationContext e) {
//        if (checkIfNotDuplicated(e.name.getText(), e)) {
//            if (ctx.name.getText().equals(e.name.getText())) {
//                this.errors.record(ParseUtil.duplicatedSymbol(ctx.name.getText(), ctx, e));
//            } else {
//                return true;
//            }
//        }
//        return false;
//    }
//
//    @Override
//    public Boolean visitDeclarationComponent(JSpearSpecificationLanguageParser.DeclarationComponentContext ctx) {
//        return super.visitDeclarationComponent(ctx);
//    }
//
//    @Override
//    public Boolean visitDeclarationPenalty(JSpearSpecificationLanguageParser.DeclarationPenaltyContext ctx) {
//        return super.visitDeclarationPenalty(ctx);
//    }
//
//    @Override
//    public Boolean visitDeclarationVariables(JSpearSpecificationLanguageParser.DeclarationVariablesContext ctx) {
//        return super.visitDeclarationVariables(ctx);
//    }
}
