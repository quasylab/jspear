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

package it.unicam.quasylab.jspear.ds;

import org.apache.commons.math3.random.RandomGenerator;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Predicate;

/**
 * This functional interface is used to model a random expression over a data state.
 */
@FunctionalInterface
public interface DataStateRandomExpression {

    double eval(RandomGenerator rg, DataState ds);

    /**
     * Returns a composed expression that applies the given operator to this expression.
     *
     * @param op double unary operator.
     * @return a composed expression that applies the given operator to this expression.
     */
    default DataStateRandomExpression apply(DoubleUnaryOperator op) {
        return (rg,ds) -> op.applyAsDouble(this.eval(rg, ds));
    }

    /**
     * Returns a composed expression that applies the given operator to this expression and to the
     * one passed as parameter.
     * @param op binary double operator.
     * @param other another expression.
     * @return a composed expression that applies the given operator to this expression and to the
     * one passed as parameter.
     */
    default DataStateRandomExpression apply(DoubleBinaryOperator op, DataStateRandomExpression other) {
        return (rg,ds) -> op.applyAsDouble(this.eval(rg, ds), other.eval(rg, ds));
    }

    /**
     * Returns a composed expression that sums this expression to the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that sums this expression to the one passed a parameter.
     */
    default DataStateRandomExpression sum(DataStateRandomExpression expr) {
        return this.apply(Double::sum, expr);
    }

    /**
     * Returns a composed expression that subtracts this expression to the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that subtracts to this expression the one passed a parameter.
     */
    default DataStateRandomExpression sub(DataStateRandomExpression expr) {
        return this.apply((d1, d2) -> d1-d2, expr);
    }

    /**
     * Returns a composed expression that multiplies this expression by the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that multiplies this expression by the one passed a parameter.
     */
    default DataStateRandomExpression mul(DataStateRandomExpression expr) {
        return this.apply((d1, d2) -> d1*d2, expr);
    }

    /**
     * Returns a composed expression that divides this expression by the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that divides this expression by the one passed a parameter.
     */
    default DataStateRandomExpression div(DataStateRandomExpression expr) {
        return this.apply((d1, d2) -> d1/d2, expr);
    }


    /**
     * Returns a composed expression that divides this expression by the given double value.
     *
     * @param x a double value.
     * @return a composed expression that divides this expression by the given double value.
     */
    default DataStateRandomExpression normalise(double x) {
        return this.apply(v -> v/x);
    }

    /**
     *
     */
    default DataStateRandomExpression ifThenElse(Predicate<DataState> predicate, DataStateRandomExpression thenExpression, DataStateRandomExpression elseExpression) {
        return (rg, ds) -> (predicate.test(ds)?thenExpression.eval(rg, ds): elseExpression.eval(rg, ds));
    }

    /**
     * Returns a random expression that is always evaluated to the same constant value v.
     *
     * @param v a constant value.
     * @return a random expression that is always evaluated to the same constant value v.
     */
    static DataStateRandomExpression constant(double v) {
        return (rg, ds) -> v;
    }


}
