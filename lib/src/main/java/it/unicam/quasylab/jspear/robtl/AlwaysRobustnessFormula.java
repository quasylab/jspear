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

package it.unicam.quasylab.jspear.robtl;

/**
 * We use the "always" operator to specify that
 * a given formula must be satisfied at each time step in a given interval.
 */
public final class AlwaysRobustnessFormula implements RobustnessFormula {

    private final RobustnessFormula formula;
    private final int from;
    private final int to;

    /**
     * The "always" formula takes three parameters:
     *
     * @param formula a RobTL formula
     * @param from the left bound of the time interval
     * @param to the right bound of the time interval.
     */
    public AlwaysRobustnessFormula(RobustnessFormula formula, int from, int to) {
        this.formula = formula;
        this.from = from;
        this.to = to;
    }

    @Override
    public <T> RobustnessFunction<T> eval(RobustnessFormulaVisitor<T> evaluator) {
        return evaluator.evalAlways(this);
    }

    /**
     * Returns the RobTL formula passed as argument to this formula.
     *
     * @return parameter <code>formula</code>.
     */
    public RobustnessFormula getArgument() {
        return this.formula;
    }

    /**
     * Returns the left bound of the time interval in this formula.
     *
     * @return parameter <code>from</code>.
     */
    public int getFrom() {
        return from;
    }

    /**
     * Returns the right bound of the time interval in this formula.
     *
     * @return parameter <code>to</code>.
     */
    public int getTo() {
        return to;
    }
}
