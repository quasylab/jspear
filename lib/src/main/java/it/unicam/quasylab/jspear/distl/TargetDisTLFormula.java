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
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import nl.tue.Monitoring.MonitoringVisitor;

import java.util.Optional;


public final class TargetDisTLFormula implements DisTLFormula, UDisTLFormula {

    private final DataStateFunction mu;

    private Optional<DataStateExpression> rho;

    private Penalty P;

    private final double q;

    public TargetDisTLFormula(DataStateFunction distribution, DataStateExpression penalty, double threshold) {
        this(distribution, threshold);
        this.P = new NonePenalty();
    }

    public TargetDisTLFormula(DataStateFunction distribution, Penalty penalty, double threshold) {
        this(distribution, threshold);
        this.rho = Optional.empty();
    }

    private TargetDisTLFormula(DataStateFunction distribution, double threshold) {
        this.mu = distribution;
        this.q = threshold;
    }


    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
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

    @Override
    public <T> T build(MonitoringVisitor<T> visitor) {
        return visitor.buildTargetMonitor(this);
    }
}
