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

package it.unicam.quasylab.jspear.controller;

import it.unicam.quasylab.jspear.ds.DataState;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Class ExecController implements a controller that
 * starts its execution in the current time step.
 */
public class ExecController implements Controller {

    private final Controller nextController;

    /**
     * Generates a controller that behaves like <code>nextController</code>
     *
     * @param nextController the controller implementing the desired behaviour
     */
    public ExecController(Controller nextController) {
        this.nextController = nextController;
    }

    /**
     * The controller applies the effect of <code>nextController</code> on the current data state
     * and then follows the behaviour of <code>nextController</code> at the next step.
     *
     * @param rg random generator
     * @param state the current data state
     * @return the effect of <code>nextController</code> on <code>state</code> and the transition to the next controller in <code>nextController</code>
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        return nextController.next(rg, state);
    }

}
