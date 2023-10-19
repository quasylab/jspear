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

package it.unicam.quasylab.jspear.distance;

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.distance.DistanceExpression;

import java.util.stream.IntStream;

/**
 * Class LinearCombinationDistanceExpression implements the convex combination of distance expressions.
 */
public final class LinearCombinationDistanceExpression implements DistanceExpression {

    public final double[] weights;
    public final DistanceExpression[] expressions;

    /**
     * Generates the convex combination of a given set of distance expressions using a given set of weights.
     * @param weights the array of the weights
     * @param expressions the array of the distance expressions.
     */
    public LinearCombinationDistanceExpression(double[] weights, DistanceExpression[] expressions) {
        if (weights.length != expressions.length) {
            throw new IllegalArgumentException();
        }
        this.weights = weights;
        this.expressions = expressions;
    }

    /**
     * Computes the convex combination of distances between two evolution sequences at a given time step.
     *
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the convex combination of the evaluations of the distance expressions between the two sequences.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        return IntStream.range(0, weights.length)
                .mapToDouble(i -> weights[i]*expressions[i].compute(step, seq1, seq2))
                .sum();
    }

    /**
     * The confidence interval is obtained from the convex combination of the confidence intervals
     * on the evaluations of the expressions.
     *
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @param m number of repetition for bootstrapping
     * @param z the desired z-score
     * @return the evaluation of the convex combination and the related confidence interval,
     * whose bounds are obtained as the convex combinations of the respective bounds from the
     * confidence intervals on the evaluations of the expressions.
     */
    @Override
    public double[] evalCI(int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z) {
        return IntStream.range(0,3).mapToDouble(j -> IntStream.range(0, weights.length)
                .mapToDouble(i -> weights[i]*expressions[i].evalCI(step, seq1, seq2, m, z)[j])
                .sum()).toArray();
    }

}
