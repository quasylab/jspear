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

package it.unicam.quasylab.jspear.perturbation;

import it.unicam.quasylab.jspear.ds.DataStateFunction;

import java.util.Optional;

/**
 * Defines the sequential composition of two perturbations that must be applied one after the other.
 *
 * @param first first perturbation to be applied.
 * @param second second perturbation to be applied.
 */
public record SequentialPerturbation(Perturbation first, Perturbation second) implements Perturbation {

    /**
     * The second perturbation is applied only once the first one has terminated.
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
     * The perturbation follows the evolution of the first perturbation until it terminates.
     * Afterward, it follows the evolution of the second perturbation.
     *
     * @return the evolution of <code>second</code> if <code>first</code> has terminated.
     * The sequential composition of the evolution of <code>first</code> with <code>second</code> otherwise.
     */
    @Override
    public Perturbation step() {
        if (first().isDone()) {
            return second.step();
        } else {
            return new SequentialPerturbation(first.step(), second);
        }
    }

    /**
     * The perturbation terminates only once both perturbation terminate.
     *
     * @return <code>true</code> if both perturbations have terminated.
     * <code>false</code> otherwise.
     */
    @Override
    public boolean isDone() {
        return first().isDone()&&second.isDone();
    }
}
