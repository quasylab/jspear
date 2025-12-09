/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
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
import org.antlr.v4.runtime.ParserRuleContext;

/**
 * This visitor is used to infer types of expressions.
 */
public class ExpressionTypeInference extends JSpearSpecificationLanguageBaseVisitor<JSpearType> {
    private final TypeEvaluationContext context;
    private final ParseErrorCollector errors;
    private final boolean randomExpressionAllowed;

    /**
     * Creates a new visitor used to infer the type of expressions. This constructor takes as parameters
     * a
     * @param context
     * @param errors
     */
    public  ExpressionTypeInference(TypeEvaluationContext context, ParseErrorCollector errors, boolean randomExpressionAllowed) {
        this.context = context;
        this.errors = errors;
        this.randomExpressionAllowed = randomExpressionAllowed;
    }

    public ExpressionTypeInference(TypeEvaluationContext context, ParseErrorCollector errors) {
        this(context, errors, false);
    }

    public static JSpearType infer(TypeEvaluationContext context, ParseErrorCollector errors, JSpearSpecificationLanguageParser.ExpressionContext value) {
        return value.accept(new ExpressionTypeInference(context, errors, true));
    }

    public static JSpearType infer(TypeEvaluationContext context, ParseErrorCollector errors, boolean randomExpressionAllowed, JSpearSpecificationLanguageParser.ExpressionContext value) {
        return value.accept(new ExpressionTypeInference(context, errors, randomExpressionAllowed));
    }


    public JSpearType inferAndCheck(JSpearType expectedType, ParserRuleContext ctx) {
        JSpearType actualType = ctx.accept(this);
        if (!expectedType.isCompatibleWith(actualType)) {
            errors.record(ParseUtil.typeError(expectedType, actualType, ctx.start));
            return JSpearType.ERROR_TYPE;
        }
        return actualType;
    }

    public boolean checkType(JSpearType expectedType, ParserRuleContext ctx) {
        return !inferAndCheck(expectedType, ctx).isError();
    }

    @Override
    public JSpearType visitNegationExpression(JSpearSpecificationLanguageParser.NegationExpressionContext ctx) {
        if (checkType(JSpearType.BOOLEAN_TYPE, ctx.arg)) {
            return JSpearType.BOOLEAN_TYPE;
        }
        return JSpearType.ERROR_TYPE;
    }

    @Override
    public JSpearType visitExponentExpression(JSpearSpecificationLanguageParser.ExponentExpressionContext ctx) {
        return combineToRealType(ctx.left, ctx.right);
    }

    private JSpearType combineToRealType(ParserRuleContext left, ParserRuleContext right) {
        JSpearType leftType = checkNumerical(left);
        JSpearType rightType = checkNumerical(right);
        return (leftType.isRandom()||rightType.isRandom()?new JSpearRandomType(JSpearType.REAL_TYPE): JSpearType.REAL_TYPE);
    }

    private JSpearType checkNumerical(ParserRuleContext ctx) {
        JSpearType type = ctx.accept(this);
        if (!type.isNumerical()) {
            errors.record(ParseUtil.expectedNumericalType(type, ctx.start));
            return JSpearType.ERROR_TYPE;
        }
        return type;
    }

    @Override
    public JSpearType visitBinaryMathCallExpression(JSpearSpecificationLanguageParser.BinaryMathCallExpressionContext ctx) {
        return combineToRealType(ctx.left, ctx.right);
    }


    @Override
    public JSpearType visitReferenceExpression(JSpearSpecificationLanguageParser.ReferenceExpressionContext ctx) {
        String name = ctx.name.getText();
        if (!context.isDefined(name)) {
            errors.record(ParseUtil.unknownSymbol(ctx.name));
            return JSpearType.ERROR_TYPE;
        }
        if (!context.isAReference(name)) {
            errors.record(ParseUtil.illegalUseOfName(ctx.name));
            return JSpearType.ERROR_TYPE;
        }
        return context.getTypeOf(name);
    }

    @Override
    public JSpearType visitIntValue(JSpearSpecificationLanguageParser.IntValueContext ctx) {
        return JSpearType.INTEGER_TYPE;
    }

    @Override
    public JSpearType visitTrueValue(JSpearSpecificationLanguageParser.TrueValueContext ctx) {
        return JSpearType.BOOLEAN_TYPE;
    }

    @Override
    public JSpearType visitRelationExpression(JSpearSpecificationLanguageParser.RelationExpressionContext ctx) {
        JSpearType leftType = ctx.left.accept(this);
        JSpearType rightType = ctx.right.accept(this);
        if (!leftType.canBeMergedWith(rightType)) {
            this.errors.record(ParseUtil.typeError(leftType,rightType, ctx.right.start));
        }
        return (leftType.isRandom()||rightType.isRandom()?new JSpearRandomType(JSpearType.BOOLEAN_TYPE):JSpearType.BOOLEAN_TYPE);
    }

    @Override
    public JSpearType visitBracketExpression(JSpearSpecificationLanguageParser.BracketExpressionContext ctx) {
        return ctx.expression().accept(this);
    }

    @Override
    public JSpearType visitOrExpression(JSpearSpecificationLanguageParser.OrExpressionContext ctx) {
        boolean isRandom = checkType(JSpearType.BOOLEAN_TYPE, ctx.left);
        checkType(JSpearType.BOOLEAN_TYPE, ctx.right);
        return JSpearType.BOOLEAN_TYPE;
    }

    @Override
    public JSpearType visitIfThenElseExpression(JSpearSpecificationLanguageParser.IfThenElseExpressionContext ctx) {
        JSpearType guardType = inferAndCheck(JSpearType.BOOLEAN_TYPE, ctx.guard);
        JSpearType thenType = ctx.thenBranch.accept(this);
        JSpearType elseType = ctx.elseBranch.accept(this);
        if (!thenType.canBeMergedWith(elseType)) {
            errors.record(ParseUtil.typeError(thenType, elseType, ctx.elseBranch.start));
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
    public JSpearType visitFalseValue(JSpearSpecificationLanguageParser.FalseValueContext ctx) {
        return JSpearType.BOOLEAN_TYPE;
    }

    @Override
    public JSpearType visitRealValue(JSpearSpecificationLanguageParser.RealValueContext ctx) {
        return JSpearType.REAL_TYPE;
    }

    @Override
    public JSpearType visitAndExpression(JSpearSpecificationLanguageParser.AndExpressionContext ctx) {
        checkType(JSpearType.BOOLEAN_TYPE, ctx.left);
        checkType(JSpearType.BOOLEAN_TYPE, ctx.right);
        return JSpearType.BOOLEAN_TYPE;
    }

    @Override
    public JSpearType visitCallExpression(JSpearSpecificationLanguageParser.CallExpressionContext ctx) {
        String functionName = ctx.name.getText();
        if (!context.isAFunction(functionName)) {
            errors.record(ParseUtil.isNotAFunction(ctx.name));
            return JSpearType.ERROR_TYPE;
        }
        JSpearType[] expectedArguments = context.getArgumentsType(functionName);
        if (ctx.callArguments.size() != expectedArguments.length) {
            errors.record(ParseUtil.illegalNumberOfArguments(ctx.name, expectedArguments.length, ctx.callArguments.size()));
            return JSpearType.ERROR_TYPE;
        }
        for(int i=0; i<expectedArguments.length; i++) {
            checkType(expectedArguments[i], ctx.callArguments.get(i));
        }
        return context.getReturnType(functionName);
    }

    @Override
    public JSpearType visitMulDivExpression(JSpearSpecificationLanguageParser.MulDivExpressionContext ctx) {
        JSpearType leftType = checkNumerical(ctx.left);
        JSpearType rightType = checkNumerical(ctx.right);
        return JSpearType.merge(leftType, rightType);
    }

    @Override
    public JSpearType visitAddSubExpression(JSpearSpecificationLanguageParser.AddSubExpressionContext ctx) {
        JSpearType leftType = checkNumerical(ctx.left);
        JSpearType rightType = checkNumerical(ctx.right);
        return JSpearType.merge(leftType, rightType);
    }

    @Override
    public JSpearType visitUnaryMathCallExpression(JSpearSpecificationLanguageParser.UnaryMathCallExpressionContext ctx) {
        checkNumerical(ctx.argument);
        return JSpearType.REAL_TYPE;
    }

    @Override
    public JSpearType visitUnaryExpression(JSpearSpecificationLanguageParser.UnaryExpressionContext ctx) {
        return checkNumerical(ctx.arg);
    }


    @Override
    public JSpearType visitNormalExpression(JSpearSpecificationLanguageParser.NormalExpressionContext ctx) {
        if (!randomExpressionAllowed) {
            this.errors.record(ParseUtil.illegalUseOfRandomExpression(ctx.start));
            return JSpearType.ERROR_TYPE;
        } else {
            if (checkType(JSpearType.REAL_TYPE, ctx.mean)&checkType(JSpearType.REAL_TYPE, ctx.variance)) {
                return new JSpearRandomType( JSpearType.REAL_TYPE );
            }
        }
        return JSpearType.ERROR_TYPE;
    }

    @Override
    public JSpearType visitUniformExpression(JSpearSpecificationLanguageParser.UniformExpressionContext ctx) {
        if (!randomExpressionAllowed) {
            this.errors.record(ParseUtil.illegalUseOfRandomExpression(ctx.start));
            return JSpearType.ERROR_TYPE;
        }
        JSpearType type = null;
        for (JSpearSpecificationLanguageParser.ExpressionContext v: ctx.values) {
            JSpearType current = v.accept(this);
            if (type == null) {
                type = current;
            } else {
                if (type.canBeMergedWith(current)) {
                    type = JSpearType.merge(type, current);
                } else {
                    this.errors.record(ParseUtil.typeError(type, current, v.start));
                    return JSpearType.ERROR_TYPE;
                }
            }
        }
        if (type == null) {
            return JSpearType.ERROR_TYPE;
        }
        return new JSpearRandomType( type );
    }

    @Override
    public JSpearType visitRandomExpression(JSpearSpecificationLanguageParser.RandomExpressionContext ctx) {
        if (!randomExpressionAllowed) {
            this.errors.record(ParseUtil.illegalUseOfRandomExpression(ctx.start));
            return JSpearType.ERROR_TYPE;
        }
        if (ctx.from != null) {
            if (checkType(JSpearType.REAL_TYPE, ctx.from)&checkType(JSpearType.REAL_TYPE, ctx.to)) {
                return new JSpearRandomType( JSpearType.REAL_TYPE );
            } else {
                return JSpearType.ERROR_TYPE;
            }
        } else {
            return new JSpearRandomType( JSpearType.REAL_TYPE );
        }
    }



}
