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
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableRegistry;
import org.antlr.v4.runtime.Token;

import java.util.List;

public class JSpearGlobalVariableCollector extends JSpearSpecificationLanguageBaseVisitor<Boolean> {

    private final JSpearVariableRegistry registry;

    private final ParseErrorCollector errors;


    public JSpearGlobalVariableCollector(ParseErrorCollector errors, JSpearVariableRegistry registry) {
        this.registry = registry;
        this.errors = errors;
    }


    @Override
    public Boolean visitDeclarationComponent(JSpearSpecificationLanguageParser.DeclarationComponentContext ctx) {
        return recordVariables(ctx.variables);
    }

    private Boolean recordVariables(List<JSpearSpecificationLanguageParser.VariableDeclarationContext> variables) {
        boolean flag = true;
        for(JSpearSpecificationLanguageParser.VariableDeclarationContext v: variables) {
            flag &= recordVariable(v.name);
        }
        return flag;
    }

    private boolean recordVariable(Token v) {
        String variableName = v.getText();
        if (registry.isDeclared(variableName)) {
            errors.record(ParseUtil.duplicatedVariablesDeclaration(v));
            return false;
        } else {
            registry.record(variableName);
            return true;
        }
    }

    @Override
    public Boolean visitDeclarationVariables(JSpearSpecificationLanguageParser.DeclarationVariablesContext ctx) {
        return recordVariables(ctx.variableDeclaration());
    }

    @Override
    public Boolean visitDeclarationConstant(JSpearSpecificationLanguageParser.DeclarationConstantContext ctx) {
        return true;
    }

    @Override
    public Boolean visitDeclarationParameter(JSpearSpecificationLanguageParser.DeclarationParameterContext ctx) {
        return true;
    }

    @Override
    public Boolean visitDeclarationFunction(JSpearSpecificationLanguageParser.DeclarationFunctionContext ctx) {
        return true;
    }

    @Override
    public Boolean visitDeclarationPenalty(JSpearSpecificationLanguageParser.DeclarationPenaltyContext ctx) {
        return true;
    }

    @Override
    public Boolean visitDeclarationEnvironmnet(JSpearSpecificationLanguageParser.DeclarationEnvironmnetContext ctx) {
        return true;
    }

    @Override
    public Boolean visitDeclarationType(JSpearSpecificationLanguageParser.DeclarationTypeContext ctx) {
        return true;
    }

    @Override
    protected Boolean defaultResult() {
        return true;
    }

    @Override
    protected Boolean aggregateResult(Boolean aggregate, Boolean nextResult) {
        return aggregate && nextResult;
    }
}

