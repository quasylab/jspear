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
import it.unicam.quasylab.jspear.SystemState;

public abstract class UDisTLMonitor<T> {

    public static final double UNDEFINED_SYMBOL = -2.0;

    protected int sampleSize;
    protected boolean parallel;
    protected final int semEvalTimestep;
    protected final DefaultRandomGenerator rg;

    public UDisTLMonitor(int semEvalTimestep, int sampleSize, boolean parallel) {
        this.sampleSize = sampleSize;
        this.semEvalTimestep = semEvalTimestep;
        this.parallel = parallel;
        rg = new DefaultRandomGenerator();
    }

    public void setRandomGeneratorSeed(int seed){
        rg.setSeed(seed);
    }

    public abstract T evalNext(SampleSet<PerceivedSystemState> sample);

    public static SampleSet<PerceivedSystemState> systemStatesToPerceivedSystemStates(SampleSet<SystemState> systemStateSample){
        return new SampleSet<>(systemStateSample.stream().map((st) -> new PerceivedSystemState(st.getDataState())).toList());
    }



}
