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

public interface UDisTLMonitor<T> {
    /**
     * Evaluates the monitored UDisTL formula on the given sample set.
     *
     * <p>This method receives a {@link SampleSet} of perceived system states
     * corresponding to the next step of the evolution, and returns the
     * monitoring output associated with that step. The returned value represents
     * the evaluation, verdict, or satisfaction information of the UDisTL formula
     * monitored by this object.</p>
     *
     * @param sample the sample set of perceived system states at the next step.
     * @return the monitoring output for the given sample set.
     */
    T evalNext(SampleSet<PerceivedSystemState> sample);
}
