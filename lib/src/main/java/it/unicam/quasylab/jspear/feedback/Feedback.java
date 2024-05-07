/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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
import it.unicam.quasylab.jspear.FeedbackSystem;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.Optional;

public sealed interface Feedback permits
        AtomicFeedback,
        DelayedFeedback,
        IterativeFeedback,
        NoneFeedback,
        SequentialFeedback,
        PersistentFeedback {

    Feedback NONE = new NoneFeedback();

    /**
     * Returns the effect of this feedback at the current time step.
     *
     * @return the effect of this feedback at the current time step.
     */
    Optional<DataStateFunction> effect();

    /**
     * Returns the feedback that will be applied at the next step.
     *
     * @return the feedback that will be applied at the next step.
     */
    Feedback next();

    /**
     * Returns true if this feedback has terminated its effects.
     *
     * @return true if this feedback has terminated its effects.
     */
    boolean isDone();


    /**
     * Applies the effect of this feedback, if present, to the current data state.
     *
     * @param rg random generator
     * @param state the current data state
     * @return <code>state</code> modified by the effect of the feedback, if there is any.
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
     * Generates a system under feedback.
     *
     * @param rg random generator
     * @param system the nominal system
     * @return a <code>PerturbedSystem</code> in which <code>this</code> perturbation is active,
     * and whose initial data state is obtained by applying the effect of <code>this</code> perturbation
     * to the current data state of <code>system</code>.
     */
    default SystemState apply(RandomGenerator rg, SystemState system) {
        return system.setDataState(this.apply(rg, system.getDataState()));
    }

}
