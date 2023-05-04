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

package it.unicam.quasylab.jspear.distl;

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;


public final class BrinkDisTLFormula implements DisTLFormula {

    private final DataStateFunction mu;

    private final DataStateExpression rho;

    private final double q;

    public BrinkDisTLFormula(DataStateFunction distribution, DataStateExpression penalty, double threshold) {
        this.mu = distribution;
        this.rho = penalty;
        this.q = threshold;
    }

    @Override
    public double eval(int sampleSize, int step, EvolutionSequence sequence, boolean parallel) {
        SampleSet<SystemState> state = sequence.get(step);
        SampleSet<SystemState> state2 = state.replica(sampleSize).applyDistribution(new DefaultRandomGenerator(),this.mu);
        double distance = state.distanceLeq(this.rho,state2);
        return distance - this.q;
    }

    @Override
    public <Double> DisTLFunction<Double> eval(DisTLFormulaVisitor<Double> evaluator) {
        return evaluator.evalBrink(this);
    }

    public DataStateFunction getDistribution() {
        return this.mu;
    }

    public DataStateExpression getPenalty() {
        return this.rho;
    }

    public double getThreshold() { return this.q; }
}
