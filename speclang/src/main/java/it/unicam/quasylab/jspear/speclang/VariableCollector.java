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

import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;

public class VariableCollector extends JSpearSpecificationLanguageBaseVisitor<JSpearVariableAllocation> {
//
//    private final VariableAllocation variableNames = new VariableAllocation();
//    private final Map<String, Double> constants;
//    private final Map<String, Double> parameters;
//
//    private String currentName = null;
//    private double currentFrom;
//    private double currentTo;
//
//    public VariableCollector(Map<String, Double> constants, Map<String, Double> parameters) {
//        this.constants = constants;
//        this.parameters = parameters;
//    }
//
//    @Override
//    public VariableAllocation visitJSpearSpecificationModel(JSpearSpecificationLanguageParser.JSpearSpecificationModelContext ctx) {
//        ctx.element().forEach(e -> e.accept(this));
//        return variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitVariablesDeclaration(JSpearSpecificationLanguageParser.VariablesDeclarationContext ctx) {
//        ctx.variableDeclaration().forEach(v -> v.accept(this));
//        return variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitVariableDeclaration(JSpearSpecificationLanguageParser.VariableDeclarationContext ctx) {
//        NumericalExpressionEvaluator ee = new NumericalExpressionEvaluator(constants, parameters);
//        this.currentName = ctx.name.getText();
//        this.currentFrom = ctx.from.accept(ee);
//        this.currentTo = ctx.to.accept(ee);
//        ctx.type().accept(this);
//        return variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitIntegerType(JSpearSpecificationLanguageParser.IntegerTypeContext ctx) {
//        this.variableNames.addVariable(currentName, currentFrom, currentTo);
//        return this.variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitRealType(JSpearSpecificationLanguageParser.RealTypeContext ctx) {
//        this.variableNames.addVariable(currentName, currentFrom, currentTo);
//        return this.variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitArrayType(JSpearSpecificationLanguageParser.ArrayTypeContext ctx) {
//        NumericalExpressionEvaluator ee = new NumericalExpressionEvaluator(constants, parameters);
//        int size = ctx.size.accept(ee).intValue();
//        this.variableNames.addArray(currentName, size, currentFrom, currentTo);
//        return this.variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitBooleanType(JSpearSpecificationLanguageParser.BooleanTypeContext ctx) {
//        this.variableNames.addVariable(currentName, currentFrom, currentTo);
//        return this.variableNames;
//    }
//
//    @Override
//    public VariableAllocation visitCustomType(JSpearSpecificationLanguageParser.CustomTypeContext ctx) {
//        this.variableNames.addVariable(currentName, currentFrom, currentTo);
//        return this.variableNames;
//    }
//
//    @Override
//    protected VariableAllocation defaultResult() {
//        return variableNames;
//    }
//
//    @Override
//    protected VariableAllocation aggregateResult(VariableAllocation aggregate, VariableAllocation nextResult) {
//        return variableNames;
//    }
}
