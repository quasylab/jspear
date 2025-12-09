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

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.ds.DataStateFunction;

import java.util.ArrayList;
import java.util.Optional;

/**
 * Class NoneFeedback implements a feedback that has no effects.
 */
public final class NoneFeedback implements Feedback {

    /**
     * The feedback has no effects.
     *
     * @return the empty effect.
     */
    @Override
    public Optional<DataStateFunction> effect() {
        return Optional.empty();
    }

    /**
     * The feedback never changes its behaviour.
     *
     * @return the feedback itself.
     */
    @Override
    public Feedback next() {
        return this;
    }

    /**
     * Since it has no effects at any time step, the feedback has terminated.
     *
     * @return true
     */
    @Override
    public boolean isDone() {
        return true;
    }

}
