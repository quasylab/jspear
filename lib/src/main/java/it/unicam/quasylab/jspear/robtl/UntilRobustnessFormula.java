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
 * We use the "until" operator to establish a correlation
 * between the satisfaction in a given time interval
 * of two given formulae:
 * the first should be satisfied until the second one is.
 */
public final class UntilRobustnessFormula implements RobustnessFormula {

    private final RobustnessFormula leftFormula;
    private final int from;
    private final int to;
    private final RobustnessFormula rightFormula;

    /**
     * The "until" formula takes four parameters:
     *
     * @param leftFormula a RobTL formula
     * @param from the left bound of the time interval
     * @param to the right bound of the time interval
     * @param rightFormula a RobTL formula.
     */
    public UntilRobustnessFormula(RobustnessFormula leftFormula, int from, int to, RobustnessFormula rightFormula) {
        if ((from<0)||(to<0)||(from>=to)) {
            throw new IllegalArgumentException();
        }
        this.leftFormula = leftFormula;
        this.from = from;
        this.to = to;
        this.rightFormula = rightFormula;
    }

    @Override
    public <T> RobustnessFunction<T> eval(RobustnessFormulaVisitor<T> evaluator) {
        return evaluator.evalUntil(this);
    }

    /**
     * Returns the RobTL formula taken as first parameter by this formula.
     *
     * @return parameter <code>leftFormula</code>.
     */
    public RobustnessFormula getLeftFormula() {
        return leftFormula;
    }

    /**
     * Returns the left of the time interval of this formula.
     *
     * @return parameter <code>from</code>.
     */
    public int getFrom() {
        return from;
    }

    /**
     * Returns the right bound of the time interval of this formula.
     *
     * @return parameter <code>to</code>.
     */
    public int getTo() {
        return to;
    }

    /**
     * Returns the RobTL formula taken as fourth parameter by this formula.
     *
     * @return parameter <code>rightFormula</code>.
     */
    public RobustnessFormula getRightFormula() {
        return rightFormula;
    }
}
