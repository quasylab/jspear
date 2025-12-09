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

import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.penalty.*;
import nl.tue.Monitoring.MonitorBuildingVisitor;

import java.util.Optional;
import java.util.OptionalInt;


public final class BrinkDisTLFormula implements DisTLFormula {

    private final DataStateFunction mu;

    private SampleSet<SystemState> dist;

    private final Optional<DataStateExpression> rho;

    private final Penalty P;

    private final double q;

    public BrinkDisTLFormula(DataStateFunction distribution, DataStateExpression penalty, double threshold) {
        this.mu = distribution;
        this.dist = new SampleSet<>();
        this.rho = Optional.of(penalty);
        this.q = threshold;
        this.P = new NonePenalty();
    }

    public BrinkDisTLFormula(DataStateFunction distribution, Penalty penalty, double threshold) {
        this.mu = distribution;
        this.dist = new SampleSet<>();
        this.rho = Optional.empty();
        this.q = threshold;
        this.P = penalty;
    }

    public BrinkDisTLFormula(SampleSet<SystemState> distribution, DataStateExpression penalty, double threshold) {
        this.mu = (rg, ds) -> ds;
        this.dist = distribution;
        this.rho = Optional.ofNullable(penalty);
        this.P = new NonePenalty();
        this.q = threshold;
    }

    public BrinkDisTLFormula(SampleSet<SystemState> distribution, Penalty penalty, double threshold) {
        this.mu = (rg, ds) -> ds;
        this.dist = distribution;
        this.rho = Optional.empty();
        this.P = penalty;
        this.q = threshold;
    }

    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
        return evaluator.evalBrink(this);
    }

    public DataStateFunction getDistribution() {
        return this.mu;
    }

    public SampleSet<SystemState> getSampledDistribution(){
        return this.dist;
    }

    public Optional<DataStateExpression> getRho() {
        return this.rho;
    }

    public Penalty getP() {
        return this.P;
    }

    public double getThreshold() { return this.q; }

    @Override
    public <T> T build(MonitorBuildingVisitor<T> visitor, int semanticsEvaluationTimestep) {
        return visitor.buildBrink(this, semanticsEvaluationTimestep);
    }

    @Override
    public int getFES() {
        return 1;
    }

    @Override
    public OptionalInt getTimeHorizon() {
        return OptionalInt.of(1);
    }
}
