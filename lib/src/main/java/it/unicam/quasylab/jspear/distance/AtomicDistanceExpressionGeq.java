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

package it.unicam.quasylab.jspear.distance;

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Class AtomicDistanceExpressionGeq implements the atomic distance expression
 * evaluating the hemidistance between the second and the first evolution sequence at a given time step.
 */
public final class AtomicDistanceExpressionGeq implements DistanceExpression {

    private final DataStateExpression rho;

    /**
     * Generates the atomic distance expression that will use the given penalty function
     * and the difference over reals for the evaluation of the ground distance on data states
     * @param rho the penalty function
     */
    public AtomicDistanceExpressionGeq(DataStateExpression rho) {
        this.rho = rho;
    }

    /**
     * Evaluates the hemidistance between the second and first evolution sequence at the given time step.
     *
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the hemidistance between the distribution reached by <code>seq2</code> and that reached by <code>seq1</code> at time <code>step</code>.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        return seq1.get(step).distanceGeq(rho, seq2.get(step));
    }

    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        double[] res = new double[3];
        res[0] = seq1.get(step).distanceGeq(rho, seq2.get(step));
        double[] partial = seq1.get(step).bootstrapDistanceGeq(rg, rho, seq2.get(step),m,z);
        res[1] = partial[0];
        res[2] = partial[1];
        return res;
    }


}
