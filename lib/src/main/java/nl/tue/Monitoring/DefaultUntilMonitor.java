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
import it.unicam.quasylab.jspear.distl.UntilDisTLFormula;

import java.util.ArrayList;
import java.util.OptionalInt;

public class DefaultUntilMonitor extends UDisTLMonitor<Double> {
    UntilDisTLFormula formula;
    private int distSeqSizeCounter;
    private double result;
    private int computationsCounter;
    ArrayList<ArrayList<UDisTLMonitor<Double>>> submonitorsL;
    ArrayList<UDisTLMonitor<Double>> submonitorsR;

    public DefaultUntilMonitor(UntilDisTLFormula formula, int semanticEvaluationTimestep, int sampleSize, boolean parallel) {
        super(semanticEvaluationTimestep, sampleSize, parallel);
        this.formula = formula;
        distSeqSizeCounter = 0;
        computationsCounter = 0;
        createSubmonitors();
    }

    private void createSubmonitors(){
        int a =  formula.getFrom();
        int b = formula.getTo();
        DefaultMonitorBuilder builder = new DefaultMonitorBuilder(sampleSize, parallel);

        submonitorsR = new ArrayList<>();
        submonitorsL = new ArrayList<>();
        for (int tau = a; tau <= b; tau++) {
            // semanticsEvaluationTimestep must be 0 for the submonitors because this monitor already discounts it in evalNext,
            // and submonitors only are fed after semanticsEvaluationTimestep has passed
            submonitorsR.add(builder.build(formula.getRightFormula(), 0));
            submonitorsL.add(new ArrayList<>());
            for (int tauprime = a; tauprime < tau; tauprime++) {
                submonitorsL.get(tau-a).add(builder.build(formula.getLeftFormula(), 0));
            }
        }
    }

    @Override
    public Double evalNext(SampleSet<PerceivedSystemState> sample) {
        distSeqSizeCounter += 1;
        int fes = formula.getFES();
        OptionalInt hrz = formula.getTimeHorizon();
        // if time horizon is present then observations past the time horizon are redundant, and the previous monitoring value
        // can be returned again
        if(hrz.isPresent() && distSeqSizeCounter > semEvalTimestep + hrz.getAsInt()){
            if (computationsCounter < hrz.getAsInt() - formula.getFES()) {
                System.out.println("Warn: Until monitor skipped computation steps");
            }
        }
        // after the fes, monitors can receive observations and produce outputs
        else if (distSeqSizeCounter >= semEvalTimestep + fes) {
            result = feedNAskSubmonitors(sample);
            computationsCounter++;
        }
        // before the fes and after timestep a, observations are valuable for the submonitors, but they cannot produce output yet
        else if (distSeqSizeCounter >= semEvalTimestep + formula.getFrom()){
            feedSubmonitors(sample);
        } else {
            result = UDisTLMonitor.UNDEFINED_SYMBOL;
        }
        return result;
    }

    private void feedSubmonitors(SampleSet<PerceivedSystemState> sample){
        feedNAskSubmonitors(sample);
    }

    private double feedNAskSubmonitors(SampleSet<PerceivedSystemState> sample){
        int a =  formula.getFrom();
        int tb = Math.min(formula.getTo(), a + distSeqSizeCounter - formula.getFES() - semEvalTimestep);
        double max = -3.0;
        if(!parallel){
            for (int tau = a; tau <= tb; tau++) {
                double evalR = submonitorsR.get(tau-a).evalNext(sample);
                double min2 = 4.0;
                for (int tauprime = a; tauprime < tau; tauprime++) {
                    double evalL = submonitorsL.get(tau-a).get(tauprime - a).evalNext(sample);
                    min2 = Math.min(min2, evalL);
                }
                double min1 = Math.min(evalR, min2);
                max = Math.max(max, min1);
            }
        } else {
            throw new UnsupportedOperationException("TODO: parallel until monitor");
        }
        return max;

    }

}
