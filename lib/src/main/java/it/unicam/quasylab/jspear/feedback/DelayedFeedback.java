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
 * Class DelayedFeedback is used to apply a given feedback after an initial time-out.
 */
public final class DelayedFeedback implements Feedback {
    private final int steps;
    private final Feedback body;

    /**
     * Delays the application of a given feedback of a given number of steps.
     *
     * @param steps number of steps in the time-out
     *              Note: since the feedback is applied when the counter reaches 1,
     *              the actual length of the time-out is of <code>steps</code> time steps.
     * @param body the feedback that will be applied after the time-out.
     */
    public DelayedFeedback(int steps, Feedback body) {
        this.steps = steps;
        this.body = body;
    }

    /**
     * During the time-out the feedback has no effect.
     * The effect of <code>body</code> will then be applied.
     *
     * @return the empty effect.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        return Optional.empty();
    }

    /**
     * Till the counter is greater than 1, we simply updated it by decreasing it by 1 at each time step.
     * Afterward, the feedback is applied.
     *
     * @return a new <code>DelayedFeedback</code> with a time-out of <code>steps-1</code> time steps if <code>steps</code> is greater than 1.
     * The feedback <code>body</code> otherwise.
     */
    @Override
    public Feedback next() {
        if (steps > 1) {
            return new DelayedFeedback(steps-1, body);
        } else {
            return body;
        }
    }

    @Override
    public boolean isDone() {
        return false;
    }

}
