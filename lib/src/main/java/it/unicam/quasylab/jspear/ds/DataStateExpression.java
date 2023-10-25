/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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


import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;
import java.util.function.ToDoubleFunction;

/**
 * This functional interface is used to model an expression over a data state.
 */
@FunctionalInterface
public interface DataStateExpression extends ToDoubleFunction<DataState> {

    @Override
    default double applyAsDouble(DataState state) {
        return eval(state);
    }

    /**
     * Returns the evaluation of this data state expression over a given data state.
     *
     * @param state a data state
     * @return the evaluation of this expression over <code>state</code>
     */
    double eval(DataState state);

    /**
     * Returns a composed expression that applies the given operator to this expression.
     *
     * @param op double unary operator.
     * @return a composed expression that applies the given operator to this expression.
     */
    default DataStateExpression apply(DoubleUnaryOperator op) {
        return ds -> op.applyAsDouble(this.eval(ds));
    }

    /**
     * Returns a composed expression that applies the given operator to this expression and to the
     * one passed as parameter.
     *
     * @param op binary double operator.
     * @param other another expression.
     * @return a composed expression that applies the given operator to this expression and to the
     * one passed as parameter.
     */
    default DataStateExpression apply(DoubleBinaryOperator op, DataStateExpression other) {
        return ds -> op.applyAsDouble(this.eval(ds), other.eval(ds));
    }

    /**
     * Returns a composed expression that sums this expression to the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that sums this expression to the one passed a parameter.
     */
    default DataStateExpression sum(DataStateExpression expr) {
        return this.apply(Double::sum, expr);
    }

    /**
     * Returns a composed expression that subtracts this expression to the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that subtracts to this expression the one passed a parameter.
     */
    default DataStateExpression sub(DataStateExpression expr) {
        return this.apply((d1, d2) -> d1-d2, expr);
    }

    /**
     * Returns a composed expression that multiplies this expression by the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that multiplies this expression by the one passed a parameter.
     */
    default DataStateExpression mul(DataStateExpression expr) {
        return this.apply((d1, d2) -> d1*d2, expr);
    }

    /**
     * Returns a composed expression that divides this expression by the one passed a parameter.
     *
     * @param expr an expression.
     * @return a composed expression that divides this expression by the one passed a parameter.
     */
    default DataStateExpression div(DataStateExpression expr) {
        return this.apply((d1, d2) -> d1/d2, expr);
    }

    /**
     * Returns a composed expression that divides this expression by the given double value.
     *
     * @param x a double value.
     * @return a composed expression that divides this expression by the given double value.
     */
    default DataStateExpression normalise(double x) {
        return this.apply(v -> v/x);
    }



}
