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

package it.unicam.quasylab.jspear;

import java.util.stream.IntStream;

public final class MaxDistanceExpression implements DistanceExpression {

    private final DistanceExpression expr1;
    private final DistanceExpression expr2;

    public MaxDistanceExpression(DistanceExpression expr1, DistanceExpression expr2) {
        this.expr1 = expr1;
        this.expr2 = expr2;
    }

    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        if (step<0) {
            throw new IllegalArgumentException();
        }
        return Math.max(expr1.compute(step, seq1, seq2), expr2.compute(step, seq1, seq2));
    }

    @Override
    public double[] evalCI(int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z) {
        if (step<0) {
            throw new IllegalArgumentException();
        }
        return IntStream.range(0,3)
                .mapToDouble(i -> Math.max(expr1.evalCI(step, seq1, seq2, m, z)[i], expr2.evalCI(step, seq1, seq2, m, z)[i]))
                .toArray();
    }
}
