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
 * An atomic perturbation is used to apply a given perturbation function to the system after a given number of time steps.
 *
 * @param afterSteps counter for the number of time steps during which the perturbation has no effect.
 *                   Note: since the perturbation is applied when the counter reaches 0,
 *                   the actual length of the time-out is of <code>afterSteps+1</code> time steps.
 * @param perturbationFunction perturbation function to apply.
 */
public record AtomicPerturbation(int afterSteps, DataStateFunction perturbationFunction) implements Perturbation {

    /**
     * If the initial time-out has passed, the perturbation function is applied.
     * Note: since the perturbation is applied when the counter reaches 0,
     * the actual length of the time-out is of <code>afterSteps+1</code> time steps.
     *
     * @return the <code>effect</code> of <code>perturbationFunction</code> is the counter <code>afterSteps</code> is 0,
     * the empty effect otherwise.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        if (afterSteps <= 0) {
            return Optional.of(perturbationFunction);
        } else {
            return Optional.empty();
        }
    }

    /**
     * The perturbation at the next step is the same perturbation with the counter decreased by 1, if the initial time-put has not yet passed,
     * or the perturbation with no effects, if it has passed.
     * Note: since the perturbation is applied when the counter reaches 0,
     * the actual length of the time-out is of <code>afterSteps+1</code> time steps.
     *
     * @return a <code>NonePerturbation</code> if the counter <code>afterStep</code> is 0,
     * an <code>AtomicPerturbation</code> having as parameters
     * the counter <code>afterStep</code> decreased by 1 and
     * this <code>perturbationFunction</code>
     * otherwise.
     */
    @Override
    public Perturbation step() {
        if (afterSteps <= 0) {
            return Perturbation.NONE;
        } else {
            return new AtomicPerturbation(afterSteps-1, perturbationFunction);
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
