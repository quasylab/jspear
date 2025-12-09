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

import java.util.Objects;
import java.util.stream.IntStream;

/**
 * Class UntilDistanceExpression implements the quantitative version of the bounded until operator
 * where existential quantification are interpreted as minima
 * and universal quantification as maxima.
 */
public final class UntilDistanceExpression implements DistanceExpression {
    private final DistanceExpression leftExpression;
    private final int from;
    private final int to;
    private final DistanceExpression rightExpression;

    /**
     * Generates an until distance expression between two distance expressions over a given time interval.
     *
     * @param leftExpression a distance expression
     * @param from the left bound of the time interval
     * @param to the right bound of the time interval
     * @param rightExpression a distance expression.
     */
    public UntilDistanceExpression(DistanceExpression leftExpression, int from, int to, DistanceExpression rightExpression) {
        this.leftExpression = Objects.requireNonNull(leftExpression);
        this.rightExpression = Objects.requireNonNull(rightExpression);
        if ((from<0)||(to<0)||(from>=to)) {
            throw new IllegalArgumentException();
        }
        this.from = from;
        this.to = to;
    }

    /**
     * For each time step t in the interval time interval, shifted by a given time step,
     * we compute the maximum between the evaluation of the right expression at time t and
     * the maximum value of the left expression up to time t,
     * and then take the minimum of these evaluations.
     *
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the minimum over t in <code>[from+step, to+step]</code> of
     * the maximum between
     * the evaluation of <code>rightExpression</code> between <code>seq1</code> and <code>seq2</code> at time <code>t</code> and
     * the maximum evaluation of <code>leftExpression</code> between <code>seq1</code> and <code>seq2</code> in the interval <code>[from+step, t)</code>.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        if (step<0) {
            throw new IllegalArgumentException();
        }
        /*
        return IntStream.range(from+step, to+step).parallel()
                .mapToDouble(i -> Math.max(rightExpression.compute(i, seq1, seq2),
                        IntStream.range(from+step,i).parallel()
                                .mapToDouble(j-> leftExpression.compute(j,seq1,seq2)).max().orElse(Double.NaN)))
                .min().orElse(Double.NaN);

         */
        double res = 1.0;
        double resL = 0.0;
        for(int i = from+step; i<to+step; i++) {
            double resR = rightExpression.compute(i, seq1, seq2);
            //double resL = leftExpression.compute(i,seq1,seq2);
            for(int j =from+step; j<i; j++) {
                double partialL = leftExpression.compute(j,seq1,seq2);
                resL = Math.max(resL, partialL);
            }
            res = Math.min(res,Math.max(resR,resL));
        }
        return res;
    }

    /**
     * @inheritDoc
     *
     * The same calculations applied to obtain the value of the distance,
     * are applied to the bounds of the confidence intervals to obtain the
     * confidence interval on the evaluation of the until distance expression.
     */
    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z) {
        if (step<0) {
            throw new IllegalArgumentException();
        }
        double[] res = {1.0,1.0,1.0};
        for(int i = from+step; i<to+step; i++) {
            double[] resR = rightExpression.evalCI(rg, i, seq1, seq2, m, z);
            double[] resL = leftExpression.evalCI(rg, i,seq1,seq2,m,z);
            for(int j =from+step; j<i; j++) {
                double[] partialL = leftExpression.evalCI(rg, j,seq1,seq2,m,z);
                resL[0] = Math.max(resL[0], partialL[0]);
                resL[1] = Math.max(resL[1], partialL[1]);
                resL[2] = Math.max(resL[2], partialL[2]);
            }
            res[0] = Math.min(res[0],Math.max(resR[0],resL[0]));
            res[1] = Math.min(res[1],Math.max(resR[1],resL[1]));
            res[2] = Math.min(res[2],Math.max(resR[2],resL[2]));
        }
        return res;
    }

}
