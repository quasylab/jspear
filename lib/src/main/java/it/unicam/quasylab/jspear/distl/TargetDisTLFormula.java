/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
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

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.penalty.*;

import java.util.Optional;


public final class TargetDisTLFormula implements DisTLFormula {

    private final DataStateFunction mu;

    private final Optional<DataStateExpression> rho;

    private final Penalty P;

    private final double q;

    public TargetDisTLFormula(DataStateFunction distribution, DataStateExpression penalty, double threshold) {
        this.mu = distribution;
        this.rho = Optional.of(penalty);
        this.q = threshold;
        this.P = new NonePenalty();
    }

    public TargetDisTLFormula(DataStateFunction distribution, Penalty penalty, double threshold) {
        this.mu = distribution;
        this.rho = Optional.empty();
        this.q = threshold;
        this.P = penalty;
    }

    @Override
    public double eval(int sampleSize, int step, EvolutionSequence sequence, boolean parallel) {
        SampleSet<SystemState> state = sequence.get(step);
        SampleSet<SystemState> state2 = state.replica(sampleSize).applyDistribution(new DefaultRandomGenerator(),this.mu);
        double distance;
        if (this.rho.isPresent()) {
            distance = state.distanceGeq(this.rho.get(), state2);
        } else {
            distance = state.distanceGeq(this.P,state2,step);
        }
        return this.q - distance;
    }

    @Override
    public <Double> DisTLFunction<Double> eval(DisTLFormulaVisitor<Double> evaluator) {
        return evaluator.evalTarget(this);
    }

    public DataStateFunction getDistribution() {
        return this.mu;
    }

    public Optional<DataStateExpression> getRho() {
        return this.rho;
    }

    public Penalty getP(){
        return this.P;
    }

    public double getThreshold() { return this.q; }
}
