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

package nl.tue.Monitoring.Default;

import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.distl.BrinkDisTLFormula;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.penalty.Penalty;
import nl.tue.Monitoring.PerceivedSystemState;

import java.util.Optional;
import java.util.OptionalDouble;

public class BrinkMonitor extends DefaultUDisTLMonitor {
    private final BrinkDisTLFormula formula;
    private int distributionSequenceSizeCounter;
    private double result;
    private boolean alreadyComputed = false;

    public BrinkMonitor(BrinkDisTLFormula formula, int semanticEvaluationTimestep, int sampleSize, boolean parallel) {
        super(semanticEvaluationTimestep, sampleSize, parallel);
        this.formula = formula;
        distributionSequenceSizeCounter = 0;
    }


    @Override
    public OptionalDouble evalNext(SampleSet<PerceivedSystemState> sample) {
        distributionSequenceSizeCounter += 1;
        if(distributionSequenceSizeCounter == semanticsEvaluationStep + formula.getFES()){
            result = computeAsSemantics(sample);
            alreadyComputed = true;
            return OptionalDouble.of(result);
        } else if(distributionSequenceSizeCounter > semanticsEvaluationStep + formula.getFES()){
            if(!alreadyComputed){
                System.out.println("Warn: Brink monitor is reporting without computing");
            }
            return OptionalDouble.of(result);
        } else {
            return OptionalDouble.empty();
        }
    }

    private double computeAsSemantics(SampleSet<PerceivedSystemState> sample) {
        DataStateFunction mu = formula.getDistribution();
        Optional<DataStateExpression> rho = formula.getRho();
        Penalty P = formula.getP();
        double q = formula.getThreshold();

        SampleSet<PerceivedSystemState> muSample;
        if (formula.getSampledDistribution().size() == 0) {
            muSample = sample.replica(sampleSize).applyDistribution(rg, mu, parallel);
        } else {
            // Turn system states into perceived system states
            muSample = new SampleSet<>(
                    formula.getSampledDistribution().stream().map((st) -> new PerceivedSystemState(st.getDataState())).toList());
        }
        return rho.map(dataStateExpression -> muSample.distanceGeq(dataStateExpression, sample) - q
        ).orElseGet(() -> muSample.distanceGeq(P, sample, semanticsEvaluationStep)) - q;
    }
}
