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

import it.unicam.quasylab.jspear.PerturbedSystem;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.Optional;

/**
 * This interface models a perturbation applied to a given sample set.
 */
public sealed interface Perturbation permits
        AfterPerturbation,
        AtomicPerturbation,
        IterativePerturbation,
        NonePerturbation,
        SequentialPerturbation,
        PersistentPerturbation {

    Perturbation NONE = new NonePerturbation();

    /**
     * Returns the effect of this perturbation at the current time step.
     *
     * @return the effect of this perturbation at the current time step.
     */
    Optional<DataStateFunction> effect();

    /**
     * Returns the perturbation that will be applied at the next step.
     *
     * @return the perturbation that will be applied at the next step.
     */
    Perturbation step();

    /**
     * Returns true if this perturbation has terminated its effects.
     *
     * @return true if this perturbation has terminated its effects.
     */
    boolean isDone();

    /**
     * Applies the effect of this perturbation, if present, to the current data state.
     *
     * @param rg random generator
     * @param state the current data state
     * @return <code>state</code> modified by the effect of the perturbation, if there is any.
     * Returns <code>state</code>, otherwise.
     */
    default DataState apply(RandomGenerator rg, DataState state) {
        Optional<DataStateFunction> effect = effect();
        if (effect.isPresent()) {
            return effect.get().apply(rg, state);
        } else {
            return state;
        }
    }

    /**
     * Generates a perturbed system affected by this perturbation.
     *
     * @param rg random generator
     * @param system the nominal system
     * @return a <code>PerturbedSystem</code> in which <code>this</code> perturbation is active,
     * and whose initial data state is obtained by applying the effect of <code>this</code> perturbation
     * to the current data state of <code>system</code>.
     */
    default SystemState apply(RandomGenerator rg, SystemState system) {
        return new PerturbedSystem(system.setDataState(this.apply(rg, system.getDataState())), this);
    }

}
