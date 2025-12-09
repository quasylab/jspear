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

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ParallelController;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageBaseVisitor;
import it.unicam.quasylab.jspear.speclang.JSpearSpecificationLanguageParser;
import it.unicam.quasylab.jspear.speclang.controller.JSpearControllerFunction;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluationFunction;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluator;
import it.unicam.quasylab.jspear.speclang.types.JSpearCustomType;
import it.unicam.quasylab.jspear.speclang.types.JSpearType;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.*;
import org.antlr.v4.runtime.Token;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;

public class JSpearModelGenerator extends JSpearSpecificationLanguageBaseVisitor<Boolean> {


    private final JSpearExpressionEvaluationContext context;

    private final Map<JSpearVariable, JSpearValue> initialValues = new HashMap<>();

    private final JSpearVariableRegistry registry = new JSpearVariableRegistry();

    private final Map<String, JSpearCustomType> customTypes = new HashMap<>();

    private final JSpearVariableAllocation allocation = new JSpearVariableAllocation();

    private final ParseErrorCollector errors;
    private BiFunction<RandomGenerator, JSpearStore, List<DataStateUpdate>> environmentFunction;

    private final ControllerRegistry controllerRegistry = new ControllerRegistry();

    private final Map<String, JSpearControllerFunction> controllerMap = new HashMap<>();

    private Controller controller = null;

    private final Map<String, Perturbation> perturbationMap = new HashMap<>();

    private final Map<String, DistanceExpression> distanceExpressionMap = new HashMap<>();

    private final Map<String, RobustnessFormula> formulaMap = new HashMap<>();

    private final Map<String, DataStateExpression> penalties = new HashMap<>();

    public JSpearModelGenerator(ParseErrorCollector errors) {
        this.errors = errors;
        this.context = new JSpearExpressionEvaluationContext(new HashMap<>());
    }


    @Override
    public Boolean visitJSpearSpecificationModel(JSpearSpecificationLanguageParser.JSpearSpecificationModelContext ctx) {
        if (!ctx.accept(new JSpearGlobalVariableCollector(this.errors, this.registry))) {
            return false;
        }
        boolean flag = true;
        for (JSpearSpecificationLanguageParser.ElementContext element: ctx.element()) {
            flag &= element.accept(this);
        }
        return flag;
    }

    @Override
    public Boolean visitDeclarationFunction(JSpearSpecificationLanguageParser.DeclarationFunctionContext ctx) {
        JSpearVariable[] localVariables = registerLocalVariables(ctx.arguments);
        JSpearExpressionEvaluationFunction bodyFunction = JSpearFunctionEvaluator.eval(context, registry, ctx.functionBlockStatement());
        context.recordFunction(ctx.name.getText(), (rg, args) -> bodyFunction.eval(rg, JSpearStore.storeOf(localVariables, args)));
        return true;
    }

    private JSpearVariable[] registerLocalVariables(List<JSpearSpecificationLanguageParser.FunctionArgumentContext> arguments) {
        return arguments.stream().map(arg -> registry.getOrRegister(arg.name.getText())).toArray(JSpearVariable[]::new);
    }

    @Override
    public Boolean visitDeclarationComponent(JSpearSpecificationLanguageParser.DeclarationComponentContext ctx) {
        String componentName = ctx.name.getText();
        for(JSpearSpecificationLanguageParser.VariableDeclarationContext v: ctx.variables) {
            recordVariable(v);
        }
        for(JSpearSpecificationLanguageParser.ControllerStateDeclarationContext state: ctx.states) {
            //String stateName = getStateName(componentName, state.name.getText());
            String stateName = state.name.getText();
            JSpearControllerFunction function = JSpearControllerStateGenerator.generate(context, registry, allocation, controllerMap, controllerRegistry, state.body);
            controllerRegistry.set(stateName, JSpearControllerFunction.toController(allocation, function));
            controllerMap.put(stateName, function);
        }
        Controller componentController = ctx.controller.accept(new JSpearControllerGenerator(controllerRegistry));
        if (controller == null) {
            controller = componentController;
        } else {
            controller = new ParallelController(controller, componentController);
        }
        return true;
    }

    public static String getStateName(String componentName, String stateName) {
        return componentName+"."+stateName;
    }

    @Override
    public Boolean visitDeclarationPenalty(JSpearSpecificationLanguageParser.DeclarationPenaltyContext ctx) {
        String penaltyName = ctx.name.getText();
        JSpearExpressionEvaluationFunction value = JSpearExpressionEvaluator.eval(context, registry, ctx.value);
        penalties.put(penaltyName, ds -> value.eval(JSpearStore.storeOf(allocation, ds)).toDouble());
        return true;
    }

    @Override
    public Boolean visitDeclarationEnvironmnet(JSpearSpecificationLanguageParser.DeclarationEnvironmnetContext ctx) {
        this.environmentFunction = ctx.block.accept(new JSpearEnvironmentGenerator(this.allocation, this.context, this.registry));
        return this.environmentFunction != null;
    }





    @Override
    public Boolean visitDeclarationType(JSpearSpecificationLanguageParser.DeclarationTypeContext ctx) {
        String typeName = ctx.name.getText();
        JSpearCustomType customType = new JSpearCustomType(typeName, ctx.elements.stream().map(t -> t.name.getText()).toArray(String[]::new));
        context.recordType(ctx.name.getText(), customType);
        customTypes.put(typeName, customType);
        return true;
    }

    @Override
    public Boolean visitDeclarationVariables(JSpearSpecificationLanguageParser.DeclarationVariablesContext ctx) {
        for(JSpearSpecificationLanguageParser.VariableDeclarationContext v: ctx.variableDeclaration()) {
            recordVariable(v);
        }
        return true;
    }

    private void recordVariable(JSpearSpecificationLanguageParser.VariableDeclarationContext v) {
        JSpearVariable variable = registry.get(v.name.getText());
        JSpearType type = getType(v.type());
        if (v.from != null) {
            double from = JSpearExpressionEvaluator.evalToValue(this.context, this.registry, v.from).toDouble();
            double to = JSpearExpressionEvaluator.evalToValue(this.context, this.registry, v.to).toDouble();
            this.allocation.add(variable, type, new DataRange(from, to));
        } else {
            this.allocation.add(variable, type);
        }
        this.initialValues.put(variable, JSpearExpressionEvaluator.evalToValue(this.context, this.registry, v.value));
    }

    private JSpearType getType(JSpearSpecificationLanguageParser.TypeContext type) {
        if (type instanceof JSpearSpecificationLanguageParser.IntegerTypeContext) {
            return JSpearType.INTEGER_TYPE;
        }
        if (type instanceof JSpearSpecificationLanguageParser.RealTypeContext) {
            return JSpearType.REAL_TYPE;
        }
        if (type instanceof JSpearSpecificationLanguageParser.BooleanTypeContext) {
            return JSpearType.INTEGER_TYPE;
        }
        if (type instanceof JSpearSpecificationLanguageParser.CustomTypeContext) {
            String typeName = ((JSpearSpecificationLanguageParser.CustomTypeContext) type).name.getText();
            if (customTypes.containsKey(typeName)) {
                return customTypes.get(typeName);
            }
        }
        return JSpearType.ERROR_TYPE;
    }

    @Override
    public Boolean visitDeclarationParameter(JSpearSpecificationLanguageParser.DeclarationParameterContext ctx) {
        return recordValue(ctx.name, ctx.expression());
    }

    @Override
    public Boolean visitDeclarationConstant(JSpearSpecificationLanguageParser.DeclarationConstantContext ctx) {
        return recordValue(ctx.name, ctx.expression());
    }

    private Boolean recordValue(Token name, JSpearSpecificationLanguageParser.ExpressionContext expression) {
        this.context.set(name.getText(), JSpearExpressionEvaluator.evalToValue(this.context, this.registry, expression));
        return true;
    }

    public SystemSpecification getSystemSpecification() {
        return new SystemSpecification(getControlledSystem(), this.penalties, this.formulaMap, this.perturbationMap, this.distanceExpressionMap);
    }

    private ControlledSystem getControlledSystem() {
        return new ControlledSystem(this.controller, getEnvironment(), getDataState());
    }

    private DataState getDataState() {
        return allocation.getDataState(initialValues);
    }

    private DataStateFunction getEnvironment() {
        return (rg, ds) -> ds.apply(this.environmentFunction.apply(rg, JSpearStore.storeOf(allocation, ds)));
    }

    @Override
    public Boolean visitDeclarationFormula(JSpearSpecificationLanguageParser.DeclarationFormulaContext ctx) {
        this.formulaMap.put(ctx.name.getText(), ctx.value.accept(new JSpearRobustnessFormulaGenerator(allocation, context, registry, perturbationMap, distanceExpressionMap, formulaMap)));
        return true;
    }

    @Override
    public Boolean visitDeclarationDistance(JSpearSpecificationLanguageParser.DeclarationDistanceContext ctx) {
        this.distanceExpressionMap.put(ctx.name.getText(), ctx.value.accept(new JSpearDistanceGenerator(allocation, context, registry, distanceExpressionMap, penalties)));
        return true;
    }

    @Override
    public Boolean visitDeclarationPerturbation(JSpearSpecificationLanguageParser.DeclarationPerturbationContext ctx) {
        this.perturbationMap.put(ctx.name.getText(), ctx.value.accept(new JSpearPerturbationGenerator(allocation, context, registry, perturbationMap)));
        return true;
    }
}
