/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *              Copyright (C) 2023.
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

package it.unicam.quasylab.jspear.feedback;


import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.ds.DataStateFunction;

import java.util.Optional;

/**
 * An atomic feedback is used to apply a given feedback function to the system after a given number of time steps.
 *
 * @param from counter for the number of steps during which the feedback has no effect.
 *             Note: since the feedback is applied when the counter reaches 0,
 *             the actual length of the time-out is of <code>from+1</code> steps.
 * @param changes function with the effects of the feedback to be applied to the data state.
 */
public record AtomicFeedback(int from, EvolutionSequence sequence, FeedbackFunction changes) implements Feedback {

    /**
     * If the initial time-out has passed, the feedback function is applied.
     * Note: since the feedback is applied when the counter reaches 0,
     * the actual length of the time-out is of <code>from+1</code> steps.
     *
     * @return the <code>effect</code> of <code>changes</code> is the counter <code>from</code> is 0,
     * the empty effect otherwise.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        if (from <= 0) {
            return Optional.of((rg, ds) -> ds.apply(changes.apply(rg, ds, sequence)));
        } else {
            return Optional.empty();
        }
    }

    /**
     * The feedback at the next step is the same feedback with the counter decreased by 1, if the initial time-put has not yet passed,
     * or the feedback with no effects, if it has passed.
     * Note: since the feedback is applied when the counter reaches 0,
     * the actual length of the time-out is of <code>from+1</code> steps.
     *
     * @return a <code>NoneFeedback</code> if the counter <code>from</code> is 0,
     * an <code>AtomicFeedback</code> having as parameters
     * the counter <code>from</code> decreased by 1 and
     * this <code>changes</code>
     * otherwise.
     */
    @Override
    public Feedback next() {
        if (from <= 0) {
            return Feedback.NONE;
        } else {
            return new AtomicFeedback(from-1,sequence,changes);
        }
    }

    /**
     * This perturbation will terminate only once the perturbation function is applied,
     * and this perturbation evolves into a perturbation with no effects.
     *
     * @return false
     */
    @Override
    public boolean isDone() {
        return false;
    }


}
