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

import it.unicam.quasylab.jspear.ds.DataStateFunction;

import java.util.Optional;

/**
 * Defines the sequential composition of two feedback that must be applied one after the other.
 *
 * @param first first feedback to be applied.
 * @param second second feedback to be applied.
 */
public record SequentialFeedback(Feedback first, Feedback second) implements Feedback {

    /**
     * The second feedback is applied only once the first one has terminated.
     *
     * @return the effect of <code>second</code> if <code>first</code> has terminated.
     * The effect of <code>first</code> otherwise.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        if (first.isDone()) {
            return second.effect();
        } else {
            return first.effect();
        }
    }

    /**
     * The feedback follows the evolution of the first feedback until it terminates.
     * Afterward, it follows the evolution of the second feedback.
     *
     * @return the evolution of <code>second</code> if <code>first</code> has terminated.
     * The sequential composition of the evolution of <code>first</code> with <code>second</code> otherwise.
     */
    @Override
    public Feedback next() {
        if (first().isDone()) {
            return second.next();
        } else {
            return new SequentialFeedback(first.next(), second);
        }
    }

    /**
     * The feedback terminates only once both feedback terminate.
     *
     * @return <code>true</code> if both feedback have terminated.
     * <code>false</code> otherwise.
     */
    @Override
    public boolean isDone() {
        return first().isDone()&&second.isDone();
    }
}
