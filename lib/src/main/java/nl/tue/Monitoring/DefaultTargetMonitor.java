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
    private final int timestep;
    private final DefaultRandomGenerator rg;

    public DefaultTargetMonitor(TargetDisTLFormula formula, int timestep, int sampleSize) {
        this(formula, timestep, sampleSize, false);
    }

    public DefaultTargetMonitor(TargetDisTLFormula formula, int timestep, int sampleSize, boolean parallel) {
        this.sampleSize = sampleSize;
        this.formula = formula;
        this.timestep = timestep;
        this.parallel = parallel;
        rg = new DefaultRandomGenerator();
    }

    public void setRandomGeneratorSeed(int seed){
        rg.setSeed(seed);
    }

    @Override
    public Double evalNext(SampleSet<PerceivedSystemState> sample) {
        DataStateFunction mu = formula.getDistribution();
        SampleSet<PerceivedSystemState> muSample = sample.replica(sampleSize).applyDistribution(rg, mu, parallel);
        Optional<DataStateExpression> rho = formula.getRho();
        Penalty P = formula.getP();
        double q = formula.getThreshold();
        if (rho.isPresent()) {
            return q - sample.distanceGeq(rho.get(), muSample);
        } else {
            return q - sample.distanceGeq(P, muSample, timestep);
        }
    }
}
