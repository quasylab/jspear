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
 * Identifies a perturbation that must be performed for a given number of times.
 *
 * @param replica number of times to repeat the perturbation.
 * @param body the perturbation to apply.
 */
public record IterativePerturbation(int replica, Perturbation body) implements Perturbation {

    /**
     * Returns the effect of the perturbation if the number of remaining iterations is positive.
     *
     * @return the effect of <code>body</code> if the counter <code>replica</code> is positive.
     * Otherwise, it returns the empty effect.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        if (replica>0) {
            return body.effect();
        } else {
            return Optional.empty();
        }
    }

    /**
     * Returns the sequential composition of the evolution of the perturbation with the remaining iterations, diminished by 1, if the counter is positive.
     * Otherwise, the iteration has terminated and the perturbation with no effects is returned.
     *
     * @return the sequential composition of the evolution of <code>body</code> with the remaining iterations decreased by 1, if the counter
     * <code>replica</code> is positive.
     * Otherwise, it returns the <code>NonePerturbation</code>.
     */
    @Override
    public Perturbation step() {
        if (replica > 0) {
            return new SequentialPerturbation(body.step(), new IterativePerturbation(replica-1, body));
        } else {
            return Perturbation.NONE;
        }
    }

    /**
     * The perturbation terminates when there are no iterations left.
     *
     * @return the boolean evaluation of <code>replica <= 0</code>.
     */
    @Override
    public boolean isDone() {
        return replica<=0;
    }
}
