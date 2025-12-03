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
import it.unicam.quasylab.jspear.distl.UntilDisTLFormula;
import nl.tue.Monitoring.PerceivedSystemState;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.stream.Stream;

public class UntilMonitor extends DefaultUDisTLMonitor {
    UntilDisTLFormula formula;
    private int distSeqSizeCounter;
    private OptionalDouble prevResult;
    private int computationsCounter;
    private final DefaultMonitorBuilder builder;
    protected ArrayList<DefaultUDisTLMonitor> submonitors2;
    protected ArrayList<DefaultUDisTLMonitor> submonitors1;


     public UntilMonitor(UntilDisTLFormula formula, int semanticEvaluationTimestep, int sampleSize, boolean parallel) {
        super(semanticEvaluationTimestep, sampleSize, parallel);
        this.formula = formula;
        distSeqSizeCounter = 0;
        computationsCounter = 0;

        builder = new DefaultMonitorBuilder(sampleSize, parallel);
        submonitors2 = new ArrayList<>();
        submonitors1 = new ArrayList<>();

    }

    @Override
    public OptionalDouble evalNext(SampleSet<PerceivedSystemState> sample) {
        distSeqSizeCounter += 1;
        int fes = formula.getFES();
        OptionalInt hrz = formula.getTimeHorizon();

        // The following three lines are a hack. The reason for it is related to the default unbounded until monitor implementation.
        // Sometimes, if you nest unbounded until operators, then each submonitor corresponding to
        // the subformulae is constructed a DefaultUntilMonitor with an until formula with a very big parameter "To".
        // When computing the horizon, these big parameters get added and the int variable may overflow to negative.
        if (hrz.isPresent() && hrz.getAsInt() < 0){
            hrz = OptionalInt.empty();
            System.out.println("Warn: Computed time horizon of until resulted negative (This maybe okay with unbounded operators)");
        }

        // if time horizon is present then observations past the time horizon are redundant, and the previous monitoring value
        // can be returned again
        if(hrz.isPresent() && distSeqSizeCounter > semanticsEvaluationStep + hrz.getAsInt()){
            if (computationsCounter < hrz.getAsInt() - formula.getFES()) {
                System.out.println("Warn: Until monitor skipped computation steps");
            }
            return prevResult;
        } else if (distSeqSizeCounter >= semanticsEvaluationStep + fes) {
            // after the fes, monitors can receive observations and produce outputs
            computationsCounter++;
            prevResult = feedNAskSubmonitors(sample);
            return prevResult;
        } else if (distSeqSizeCounter > semanticsEvaluationStep + formula.getFrom()){
            // after the semEvalTimeStep and from, observations are valuable for the submonitors, but they cannot produce output yet
            feedSubmonitors(sample);
        }
        // before fes, monitor cannot do anything besides returning empty
        return OptionalDouble.empty();
    }

    private void feedSubmonitors(SampleSet<PerceivedSystemState> sample){
        feedNAskSubmonitors(sample);
    }

    private OptionalDouble feedNAskSubmonitors(SampleSet<PerceivedSystemState> sample) {
         int a =  formula.getFrom();
         int b = formula.getTo();
         // distSeqSizeCounter counts every input sample set, including samples before semEvalTimestep, but the definition of |S|
         // assumes a  semEvalTimestep = 0, so it must be subtracted
         int lengthS = distSeqSizeCounter - semanticsEvaluationStep;
         int tb = Math.min(b, a + lengthS - formula.getFES());

        // 1. create monitors
        // For each new sample, distSeqSizeCounter (a proxy for |S|) is incremented by one, and thus tau and tau' have their
        // ranges increased by one. This means a new submonitor for both formulae is needed.
        // semanticsEvaluationTimestep must be 0 for the submonitors because this monitor already discounts it in evalNext,
        // and submonitors only are fed after semanticsEvaluationTimestep has passed.

        submonitors2.add(builder.build(formula.getRightFormula(), 0));
        submonitors1.add(builder.build(formula.getLeftFormula(), 0));

        // 2. feed monitors
        // The monitor submonitor1.get(i) corresponds to m[phi1](S[tau:]) for tau = semEvalTimestep + formula.getFrom() + i
        // The monitor submonitor2.get(i) corresponds to m[phi2](S[tau:]) for tau = semEvalTimestep + formula.getFrom() + i

        Stream<OptionalDouble> evals2stream = maybeParallelize(submonitors2.stream()).map((submonitor) -> submonitor.evalNext(sample));
        Stream<OptionalDouble> evals1stream = maybeParallelize(submonitors1.stream()).map((submonitor) -> submonitor.evalNext(sample));

        List<OptionalDouble> evals2 = evals2stream.toList();
        List<OptionalDouble> evals1 = evals1stream.toList();

        // 3. compute the max of mins
        int tauRange = tb - a;
        return maybeParallelizedIntRange(0,tauRange + 1).mapToDouble((tau) -> {
            OptionalDouble eval2 = evals2.get(tau);
            if (eval2.isEmpty()){
                throw new RuntimeException("Until monitor feeds and asks but submonitors (2) cannot respond yet!");
            }
            if (tau == 0){
                return eval2.getAsDouble();
            } else {
                OptionalDouble eval1 = maybeParallelizedIntRange(0, tau).mapToDouble((tauprime) -> evals1.get(tauprime).orElse(Double.NaN)).min();
                if (eval1.isEmpty() || Double.isNaN(eval1.getAsDouble())) {
                    throw new RuntimeException("Until monitor feeds and asks but submonitors (2) cannot respond yet!");
                }
                return Math.min(eval2.getAsDouble(), eval1.getAsDouble());
            }
        } ).max();
    }



}
