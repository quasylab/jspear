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

import it.unicam.quasylab.jspear.distl.UntilDisTLFormula;
import it.unicam.quasylab.jspear.udistl.UnboundedUntiluDisTLFormula;

public class UnboundedUntilMonitor extends UntilMonitor {

    // In practice, an unbounded until monitor that needs memory to scale with the length of the observed trace (i.e.
    // this implementation) can be reduced to an until monitor with the "to" parameter set to a very big number.
    // This ensures that the bounded monitoring computation uses tb = a + distSeqSizeCounter - formula.getFES() - semEvalTimestep
    // instead of tb = formula.getTo().
    // Then, the until monitor evaluation is equal to the unbounded monitor evaluation, up until the moment where there is not
    // enough memory to keep storing a VERY_BIG_NUMBER_THAT_WONT_OVERFLOW amount of information.

    // I take the maximum integer value, make it even, and divide it by two. This yields 1073741823. The reason I divide it
    // is that being to close to Integer.MAX_VALUE risks certain computations like time horizon to overflow to negative.
    // So Integer.MAX_VALUE/2 is still big but less likely to overflow.
    private static final int VERY_BIG_NUMBER_THAT_WONT_OVERFLOW = (Integer.MAX_VALUE-1)/2;

    public UnboundedUntilMonitor(UnboundedUntiluDisTLFormula formula, int semanticEvaluationTimestep, int sampleSize, boolean parallel) {
        super(new UntilDisTLFormula(formula.getLeftFormula(), 0, VERY_BIG_NUMBER_THAT_WONT_OVERFLOW,
                formula.getRightFormula()), semanticEvaluationTimestep, sampleSize, parallel);
    }
}
