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
import java.util.function.DoubleBinaryOperator;
import java.util.function.ToDoubleBiFunction;
import java.util.stream.IntStream;

/**
 * Class AtomicDistanceExpression implements the atomic distance expression
 * evaluating the Wasserstein lifting of the ground distance, obtained from
 * the given penalty function over data states and the given distance over reals,
 * between the distributions reached at a given time step
 * by two given evolution sequences.
 */
public final class AtomicDistanceExpression implements DistanceExpression {

    private final DataStateExpression rho;
    private final DoubleBinaryOperator distance;

    /**
     * Generates the atomic distance expression that will use the given penalty function
     * and the given distance over reals for the evaluation of the ground distance on data states.
     * @param rho the penalty function
     * @param distance ground distance on reals.
     */
    public AtomicDistanceExpression(DataStateExpression rho, DoubleBinaryOperator distance) {
        this.rho = rho;
        this.distance = distance;
    }

    /**
     * Evaluates the Wasserstein lifting of this ground distance
     * between the distributions reached at a given time step
     * by two given evolution sequences.
     *
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the Wasserstein lifting of the ground distance over data states obtained
     * from <code>this.distance</code> and <code>this.rho</code> between
     * the distribution reached by <code>seq1</code> and that reached by <code>seq2</code>
     * at time <code>step</code>.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        return seq1.get(step).distance(this.rho, this.distance, seq2.get(step));
    }

    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        double[] res = new double[3];
        res[0] = seq1.get(step).distance(this.rho, this.distance, seq2.get(step));
        ToDoubleBiFunction<double[],double[]> bootDist = (a,b)->IntStream.range(0, a.length).parallel()
                .mapToDouble(i -> IntStream.range(0, b.length/a.length).mapToDouble(j -> distance.applyAsDouble(a[i],b[i * (b.length/a.length) + j])).sum())
                .sum() / b.length;
        double[] partial = seq1.get(step).bootstrapDistance(rg, this.rho, bootDist, seq2.get(step),m,z);
        res[1] = partial[0];
        res[2] = partial[1];
        return res;
    }
}
