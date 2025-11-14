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

import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.distl.TargetDisTLFormula;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.penalty.Penalty;

import java.util.Optional;

public class DefaultTargetMonitor extends UDisTLMonitor<Double> {

    private final TargetDisTLFormula formula;
    private int distributionSequenceSizeCounter;
    private double result;
    private boolean alreadyComputed = false;

    public DefaultTargetMonitor(TargetDisTLFormula formula, int semanticEvaluationTimestep, int sampleSize, boolean parallel) {
        super(semanticEvaluationTimestep, sampleSize, parallel);
        this.formula = formula;
        distributionSequenceSizeCounter = 0;
    }


    @Override
    public Double evalNext(SampleSet<PerceivedSystemState> sample) {
        distributionSequenceSizeCounter += 1;
        if(distributionSequenceSizeCounter == semEvalTimestep + formula.getFES()){
            result = computeAsSemantics(sample);
            alreadyComputed = true;
            return result;
        } else if(distributionSequenceSizeCounter > semEvalTimestep + formula.getFES()){
            if(!alreadyComputed){
                System.out.println("Warn: Target monitor is reporting without computing");
            }
            return result;
        } else {
            return UDisTLMonitor.UNDEFINED_SYMBOL;
        }
    }

    private double computeAsSemantics(SampleSet<PerceivedSystemState> sample){
        DataStateFunction mu = formula.getDistribution();
        SampleSet<PerceivedSystemState> muSample = sample.replica(sampleSize).applyDistribution(rg, mu, parallel);
        Optional<DataStateExpression> rho = formula.getRho();
        Penalty P = formula.getP();
        double q = formula.getThreshold();
        if (rho.isPresent()) {
            return q - sample.distanceGeq(rho.get(), muSample);
        } else {
            return q - sample.distanceGeq(P, muSample, semEvalTimestep);
        }
    }
}
