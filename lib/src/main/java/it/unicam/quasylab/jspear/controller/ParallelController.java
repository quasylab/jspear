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
 * Class ParallelController implements a controller consisting of two controllers running synchronously in parallel.
 * At each step the effects and transitions of both are applied.
 */
public class ParallelController implements Controller {

    private final Controller leftController;
    private final Controller rightController;

    /**
     * Creates a new controller consisting of the parallel composition of the two given controllers.
     *
     * @param leftController a controller.
     * @param rightController a controller.
     */
    public ParallelController(Controller leftController, Controller rightController) {
        this.leftController = leftController;
        this.rightController = rightController;
    }

    /**
     * Defines the effect of a ParallelController:
     * the effects of both controllers are applied to the current data state
     * and the controller at the next step is given by the parallel composition of the next step controllers
     *
     * @param rg random generator
     * @param state the current data state
     * @return the concatenation of the effects of <code>leftController</code> and <code>rightController</code> on <code>state</code>
     * and the parallel composition of the respective behaviours at the next step.
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        return this.leftController.next(rg, state).parallel(ParallelController::new, this.rightController.next(rg, state));
    }
}
