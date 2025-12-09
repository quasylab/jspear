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
 * Class NonePerturbation implements a perturbation that has no effects.
 */
public final class NonePerturbation implements Perturbation {

    /**
     * The perturbation has no effects.
     *
     * @return the empty effect.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        return Optional.empty();
    }

    /**
     * The perturbation never changes its behaviour.
     *
     * @return the perturbation itself.
     */
    @Override
    public Perturbation step() {
        return this;
    }

    /**
     * Since it has no effects at any time step, the perturbation has terminated.
     *
     * @return true
     */
    @Override
    public boolean isDone() {
        return true;
    }
}
