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

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.stream.IntStream;

/**
 * Distance expressions are used for the definition of distances between evolution sequences.
 * The interface also offers the methods for their evaluation.
 */
public sealed interface DistanceExpression permits
        AtomicDistanceExpression,
        AtomicDistanceExpressionLeq,
        AtomicDistanceExpressionGeq,
        ConvexCombinationDistanceExpression,
        MaxDistanceExpression,
        MaxIntervalDistanceExpression,
        MinDistanceExpression,
        MinIntervalDistanceExpression,
        UntilDistanceExpression,
        ThresholdDistanceExpression {

    /**
     * Returns the evaluation of the distance expression between the two sequences at the given step.
     *
     * @param step time step at which we evaluate the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the evaluation of the distance expression at the given step between the two sequences.
     */
    double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2);

    /**
     * Returns the evaluation of the distance expression between the two sequences at each time step in a given interval.
     *
     * @param from left bound of the time interval
     * @param to right bound of the time interval
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the array containing the evaluations of the distance expression between <code>seq1</code> and <code>seq2</code>
     * at each time step in <code>[from,to]</code>
     */
    default double[] compute(int from, int to, EvolutionSequence seq1, EvolutionSequence seq2) {
        return compute(IntStream.range(from, to+1).toArray(), seq1, seq2);
    }

    /**
     * Returns the evaluation of the distance expression between the two sequences at each time step in a given interval.
     *
     * @param steps time interval
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the array containing the evaluations of the distance expression between <code>seq1</code> and <code>seq2</code>
     * at each time step in <code>steps</code>.
     */
    default double[] compute(int[] steps, EvolutionSequence seq1, EvolutionSequence seq2) {
        return IntStream.of(steps).mapToDouble(i -> compute(i, seq1, seq2)).toArray();
    }

    /**
     * Returns the evaluation of the distance expression among the two sequences at the given step
     * and the related confidence interval with respect to a desired coverage probability.
     *
     * @param rg random generator
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @param m number of repetitions for the bootstrap method
     * @param z the quantile of the standard normal distribution corresponding to the desired coverage probability.
     * @return the evaluation of the distance expression,
     * at time <code>step</code>,
     * between <code>seq1</code> and <code>seq2</code>,
     * and its confidence interval evaluated via empirical bootstrapping
     * using <code>m</code> and <code>z</code> as parameters for it.
     */
    double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z);

    /**
     * In case the random generator is not declared,
     * the default one is used.
     *
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @param m number of repetitions for the bootstrap method
     * @param z the quantile of the standard normal distribution corresponding to the desired coverage probability.
     * @return the evaluation of the distance expression at the given step among the two sequences and its confidence interval.
     */

    default double[] evalCI(int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        return evalCI(new DefaultRandomGenerator(), step, seq1, seq2, m, z);
    }

}
