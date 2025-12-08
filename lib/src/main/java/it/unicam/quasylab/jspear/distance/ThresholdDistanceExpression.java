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
import it.unicam.quasylab.jspear.ds.RelationOperator;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Class ThresholdDistanceExpression implements conditional distance expressions.
 */
public final class ThresholdDistanceExpression implements DistanceExpression {

    private final double threshold;
    private final RelationOperator relop;
    private final DistanceExpression expression;

    /**
     * Generates a conditional distance expression that compares the evaluation of a given distance expression
     * with a given threshold according to a given relation operator.
     *
     * @param expression the distance expression
     * @param relop a relation operator
     * @param threshold the threshold value
     */
    public ThresholdDistanceExpression(DistanceExpression expression, RelationOperator relop, double threshold) {
        this.expression = expression;
        this.relop = relop;
        this.threshold = threshold;
    }


    /**
     * Computes the conditional distance expression by evaluating, at a given time step,
     * the distance expression between two evolution sequences.
     *
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return 0.0 if the evaluation of <code>expression</code> between <code>seq1</code> and <code>seq2</code> at time <code>step</code>
     * is in relation <code>relop</code> with <code>threshold</code>.
     * Returns 1.0 otherwise.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        return (relop.eval(expression.compute(step, seq1, seq2),threshold)?0.0:1.0);
    }

    /**
     * @inheritDoc
     *
     * If the threshold falls within the confidence interval for the distance expression,
     * then the confidence interval is set as the entire interval [0,1].
     * Otherwise, it is reduced to coincide with the evaluation of the conditional distance expression
     * (i.e., it is either [0,0] or [1,1]).
     */
    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z) {
        double[] res = new double[3];
        double[] value = expression.evalCI(rg, step, seq1, seq2, m, z);
        res[0]= relop.eval(value[0],threshold)?0.0:1.0;
        if(value[1]< threshold && threshold< value[2]){
            res[1] = 0.0;
            res[2] = 1.0;
        }
        else {
            res[1] = res[0];
            res[2] = res[0];
        }
        return res;
    }

}
