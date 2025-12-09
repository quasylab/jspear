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

import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.types.JSpearCustomType;
import it.unicam.quasylab.jspear.speclang.types.JSpearType;
import it.unicam.quasylab.jspear.speclang.types.TypeEvaluationContext;
import org.antlr.v4.runtime.ParserRuleContext;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.stream.Stream;

public class SymbolTable {

    private final Map<String, ParserRuleContext> symbols = new HashMap<>();
    private final Map<String, JSpearSpecificationLanguageParser.DeclarationFunctionContext> functions = new HashMap<>();

    private final Map<String, JSpearSpecificationLanguageParser.VariableDeclarationContext> variables = new HashMap<>();

    private final Map<String, JSpearSpecificationLanguageParser.DeclarationConstantContext> constants = new HashMap<>();

    private final Map<String, JSpearSpecificationLanguageParser.DeclarationPenaltyContext> penalties = new HashMap<>();

    private final Map<String, JSpearSpecificationLanguageParser.DeclarationComponentContext> components = new HashMap<>();


    private final Map<String, JSpearType> typesOfRefereneableElements = new HashMap<>();
    private final Map<String, JSpearType[]> functionArguments = new HashMap<>();
    private final Map<String, JSpearType> functionReturnTypes = new HashMap<>();
    private final Map<String, JSpearType> custumTypes = new HashMap<>();

    private final Set<String> elementsOfDeclaredTypes = new HashSet<>();

    public ParserRuleContext get(String name) {
        return symbols.get(name);
    }

    public void recordFunction(String name, JSpearType[] arguments, JSpearType returnType, JSpearSpecificationLanguageParser.DeclarationFunctionContext ctx) {
        if (symbols.containsKey(name)) {
            throw new IllegalArgumentException();
        }
        this.symbols.put(name, ctx);
        this.functions.put(name, ctx);
        this.functionArguments.put(name, arguments);
        this.functionReturnTypes.put(name, returnType);
    }

    public JSpearSpecificationLanguageParser.DeclarationFunctionContext getFunctionDeclaration(String name) {
        return this.functions.get(name);
    }


    private boolean isTypeElement(String name) {
        return this.elementsOfDeclaredTypes.contains(name);
    }

    public boolean isAConstant(String name) {
        return constants.containsKey(name);
    }

    public boolean isAVariable(String name) {
        return variables.containsKey(name);
    }


    public boolean isAFunction(String functionName) {
        return this.functions.containsKey(functionName);
    }

    public void recordConstant(String name, JSpearType type, JSpearSpecificationLanguageParser.DeclarationConstantContext ctx) {
        if (symbols.containsKey(name)) {
            throw new IllegalArgumentException();
        }
        this.symbols.put(name, ctx);
        this.constants.put(name, ctx);
        this.typesOfRefereneableElements.put(name, type);
    }

    public void recordVariable(String name, JSpearType type, JSpearSpecificationLanguageParser.VariableDeclarationContext ctx) {
        if (symbols.containsKey(name)) {
            throw new IllegalArgumentException();
        }
        this.symbols.put(name, ctx);
        this.variables.put(name, ctx);
        this.typesOfRefereneableElements.put(name, type);
    }

    public void recordPenaltyFunction(String name, JSpearSpecificationLanguageParser.DeclarationPenaltyContext ctx) {
        if (symbols.containsKey(name)) {
            throw new IllegalArgumentException();
        }
        this.symbols.put(name, ctx);
        this.penalties.put(name, ctx);
    }

    public void recordComponent(String name, JSpearSpecificationLanguageParser.DeclarationComponentContext ctx) {
        if (symbols.containsKey(name)) {
            throw new IllegalArgumentException();
        }
        this.symbols.put(name, ctx);
        this.components.put(name, ctx);
    }

    public boolean isDefined(String name) {
        return this.symbols.containsKey(name);
    }


    public void recordCustomType(JSpearSpecificationLanguageParser.DeclarationTypeContext ctx) {
        String customTypeName = ctx.name.getText();
        String[] customTypeElements = ctx.elements.stream().map(e -> e.name.getText()).toArray(String[]::new);
        if (isDefined(customTypeName)|| Stream.of(customTypeElements).anyMatch(this::isDefined)) {
            throw new IllegalArgumentException();
        }
        this.symbols.put(customTypeName, ctx);
        ctx.elements.forEach(e -> this.symbols.put(e.name.getText(), e));
        JSpearType customType = new JSpearCustomType(customTypeName, customTypeElements);
        this.custumTypes.put(customTypeName, customType);
        Stream.of(customTypeElements).forEach(e -> {
            this.typesOfRefereneableElements.put(e, customType);
            this.elementsOfDeclaredTypes.add(e);
        });
    }

    public boolean isACustomType(String typeName) {
        return this.custumTypes.containsKey(typeName);
    }

    public JSpearType getCustomType(String typeName) {
        return this.custumTypes.get(typeName);
    }

}
