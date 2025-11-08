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

package nl.tue.Monitoring;

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.distl.TargetDisTLFormula;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.penalty.Penalty;

import java.util.Optional;

public class DefaultTargetMonitor implements UDisTLMonitor<Double> {
    int sampleSize;
    TargetDisTLFormula formula;
    boolean parallel;

    public DefaultTargetMonitor(TargetDisTLFormula formula, int sampleSize) {
        this.sampleSize = sampleSize;
        this.formula = formula;
        parallel = false;
    }

    public DefaultTargetMonitor(TargetDisTLFormula formula,  int sampleSize, boolean parallel) {
        this.sampleSize = sampleSize;
        this.formula = formula;
        this.parallel = parallel;
    }

    @Override
    public Double evalNext(SampleSet<PerceivedSystemState> sample) {
        DataStateFunction mu = formula.getDistribution();
        SampleSet<PerceivedSystemState> muSample = sample.replica(sampleSize).applyDistribution(new DefaultRandomGenerator(), mu);
        Optional<DataStateExpression> rho = formula.getRho();
        Penalty P = formula.getP();
        double q = formula.getThreshold();
        if (rho.isPresent()) {
            return q - sample.distanceGeq(rho.get(), muSample);
        } else {
            return q - sample.distanceGeq(P, muSample, 0);
        }
    }
}
