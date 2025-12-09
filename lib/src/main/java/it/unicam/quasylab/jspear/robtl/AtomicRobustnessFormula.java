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

import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.ds.RelationOperator;

/**
 * Assuming an evolution sequence and a time step,
 * we use the atomic formula to evaluate the distance,
 * specified by a given expression,
 * between the evolution sequence and its perturbation,
 * obtained by applying a given perturbation from the time step,
 * and to compare it with a given threshold.
 */
public final class AtomicRobustnessFormula implements RobustnessFormula {

    private final Perturbation perturbation;
    private final DistanceExpression expr;
    private final RelationOperator relop;
    private final double threshold;

    /**
     * An atomic formula takes four parameters:
     *
     * @param perturbation the perturbation that we want to apply to the evolution sequence
     * @param expr the distance expression that we want to evaluate
     * @param relop a relation operator to compare the distance with the given threshold
     * @param threshold the threshold
     */
    public AtomicRobustnessFormula(Perturbation perturbation, DistanceExpression expr, RelationOperator relop, double threshold) {
        this.perturbation = perturbation;
        this.expr = expr;
        this.relop = relop;
        this.threshold = threshold;
    }

    @Override
    public <T> RobustnessFunction<T> eval(RobustnessFormulaVisitor<T> evaluator) {
        return evaluator.evalAtomic(this);
    }

    /**
     * Returns the distance expression specified in this formula.
     *
     * @return parameter <code>expr</code>.
     */
    public DistanceExpression getDistanceExpression() {
        return this.expr;
    }

    /**
     * Returns the perturbation specified in this formula.
     *
     * @return parameter <code>perturbation</code>.
     */
    public Perturbation getPerturbation() {
        return this.perturbation;
    }

    /**
     * Returns the relation operator specified in this formula.
     *
     * @return parameter <code>relop</code>.
     */
    public RelationOperator getRelationOperator() {
        return this.relop;
    }

    /**
     * Returns the threshold specified in this formula.
     *
     * @return parameter <code>threshold</code>.
     */
    public double getThreshold() {
        return this.threshold;
    }

}
