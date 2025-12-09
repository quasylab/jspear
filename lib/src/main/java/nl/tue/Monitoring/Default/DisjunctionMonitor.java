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
import it.unicam.quasylab.jspear.distl.DisjunctionDisTLFormula;
import nl.tue.Monitoring.PerceivedSystemState;

import java.util.OptionalDouble;

public class DisjunctionMonitor extends DefaultUDisTLMonitor {
    final DefaultUDisTLMonitor submonitorL;
    final DefaultUDisTLMonitor submonitorR;

    public DisjunctionMonitor(DisjunctionDisTLFormula formula, int formulaEvalTimestep, int sampleSize, boolean parallel) {
        super(formulaEvalTimestep, sampleSize, parallel);
        DefaultMonitorBuilder builder = new DefaultMonitorBuilder(sampleSize, parallel);
        submonitorL = builder.build(formula.getLeftFormula(), formulaEvalTimestep);
        submonitorR = builder.build(formula.getRightFormula(), formulaEvalTimestep);
    }

    @Override
    public OptionalDouble evalNext(SampleSet<PerceivedSystemState> sample) {
        OptionalDouble evalL = submonitorL.evalNext(sample);
        OptionalDouble evalR = submonitorR.evalNext(sample);
        if(evalL.isEmpty() || evalR.isEmpty()){
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(Math.max(evalL.getAsDouble(), evalR.getAsDouble()));
    }
}
