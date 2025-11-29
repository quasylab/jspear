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

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.SampleSet;
import nl.tue.Monitoring.PerceivedSystemState;
import nl.tue.Monitoring.UDisTLMonitor;

import java.util.OptionalDouble;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public abstract class DefaultUDisTLMonitor implements UDisTLMonitor<OptionalDouble> {

    protected int sampleSize;
    protected boolean parallel;
    protected final int semanticsEvaluationStep;
    protected final DefaultRandomGenerator rg;

    public DefaultUDisTLMonitor(int semanticsEvaluationStep, int sampleSize, boolean parallel) {
        this.sampleSize = sampleSize;
        this.semanticsEvaluationStep = semanticsEvaluationStep;
        this.parallel = parallel;
        rg = new DefaultRandomGenerator();
    }

    /**
     * Processes the next sample set of perceived system states and returns the
     * corresponding monitoring output.
     *
     * <p>This method advances the internal state of the monitor by consuming the
     * next {@link SampleSet} in the evolution.</p>
     *
     * <p>Formally, a {@link DefaultUDisTLMonitor} object represents {@code m[φ]}, where {@code φ} is a DisTL or uDisTL formula.
     * We assume that {@code sample} represents a set of datastates drawn from a distribution {@code s}.
     * The call {@code monitor.evalNext(sample)} returns {@code OptionalDouble.empty()} if the method {@code monitor.evalNext}
     * has been called less than {@code fes(φ)} times. If {@code semanticsEvaluationStep} is different from 0, then
     * {@code OptionalDouble.empty()} is returned if the method {@code monitor.evalNext} has been called less than {@code semanticsEvaluationStep+fes(φ)}.</p>
     *
     * <p>Otherwise, for a newly-created {@link DefaultUDisTLMonitor} object {@code monitor}, the result of {@code monitor.evalNext(sample)} is equal
     * to {@code m[φ](s)}. If the method {monitor.evalNext} has been called before with parameters {@code s0,s1...sn}, then
     * {@code monitor.evalNext(sample)} is equal to {@code m[φ](s0,s1...sn,s)}.</p>
     *
     * @param sample the next sample set of perceived system states.
     * @return the monitoring output generated after processing the given sample.
     */
    @Override
    abstract public OptionalDouble evalNext(SampleSet<PerceivedSystemState> sample);

    public void setRandomGeneratorSeed(int seed){
        rg.setSeed(seed);
    }

    protected <P> Stream<P> maybeParallelize(Stream<P> s){
        if (parallel){
            return s.parallel();
        }
        return s.sequential();
    }

    protected IntStream maybeParallelizedIntRange(int startInclusive, int endExclusive){
        if (parallel){
            return IntStream.range(startInclusive, endExclusive).parallel();
        }
        return  IntStream.range(startInclusive, endExclusive).sequential();
    }

}
