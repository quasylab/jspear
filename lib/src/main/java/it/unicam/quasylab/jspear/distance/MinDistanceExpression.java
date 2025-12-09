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
import org.apache.commons.math3.random.RandomGenerator;

import java.util.stream.IntStream;

/**
 * Class MinDistanceExpression implements the evaluation of the minimum between two given distance expressions.
 */
public final class MinDistanceExpression implements DistanceExpression {

    private final DistanceExpression expr1;
    private final DistanceExpression expr2;

    /**
     * Generates the distance expression for the evaluation of the minimum of the two given distance expressions.
     *
     * @param expr1 a distance expression
     * @param expr2 a distance expression.
     */
    public MinDistanceExpression(DistanceExpression expr1, DistanceExpression expr2) {
        this.expr1 = expr1;
        this.expr2 = expr2;
    }

    /**
     * Computes the two expressions between two given evolution sequences at a given time steps
     * and then evaluates the minimum.
     *
     * @param step step at which the expression is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the minimum between the evaluation of <code>expr1</code> between <code>seq1</code> and <code>seq2</code> at time <code>step</code>
     * and the evaluation of <code>expr2</code> between <code>seq1</code> and <code>seq2</code> at time <code>step</code>.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        if (step<0) {
            throw new IllegalArgumentException();
        }
        return Math.min(expr1.compute(step, seq1, seq2), expr2.compute(step, seq1, seq2));
    }

    /**
     * @inheritDoc
     *
     * The confidence interval is obtained from the confidence intervals on the evaluations of the two expressions
     * by taking the minima of the respective bounds.
     */
    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z) {
        if (step<0) {
            throw new IllegalArgumentException();
        }
        return IntStream.range(0,3)
                .mapToDouble(i -> Math.min(expr1.evalCI(rg, step, seq1, seq2, m, z)[i], expr2.evalCI(rg, step, seq1, seq2, m, z)[i]))
                .toArray();
    }

}
