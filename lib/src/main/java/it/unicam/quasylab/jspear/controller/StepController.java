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

import java.util.LinkedList;
import java.util.function.ToIntBiFunction;

/**
 * Class StepController implements a controller that
 * idles for a given number of steps
 * and then evolves to another controller
 */
public class StepController implements Controller {

    private final ToIntBiFunction<RandomGenerator, DataState> steps;
    private final Controller nextController;

    /**
     * In case the number of steps is not specified, it is assigned the default value 0
     * @param nextController the controller after the idling period
     */
    public StepController(Controller nextController) {
        this(0, nextController);
    }

    /**
     * In case the number of steps is passed as an integer
     * it is transformed into a binary function to integers
     * @param steps the duration of the idling period in time steps
     * @param nextController the controller after the idling period
     */
    public StepController(int steps, Controller nextController) {
        this((rg, ds) -> steps, nextController);
    }

    /**
     * Generates a controller that idles for <code>steps</code> time steps
     * and then behaves like <code>nextController</code>
     * @param steps the duration of the idling period in time steps
     * @param nextController the controller after the idling period
     */
    public StepController(ToIntBiFunction<RandomGenerator, DataState> steps, Controller nextController) {
        this.steps = steps;
        this.nextController = nextController;
    }

    /**
     * If the number of steps is 0, the effect and transition of <code>nextController</code> are immediately applied.
     * Otherwise, the controller has no effect for <code>step</code> time steps,
     * and then behaves like <code>nextController</code>.
     *
     * @param rg random generator
     * @param state the current data state
     * @return the effect and transition of <code>nextController</code> after a time-out of <code>steps</code> time steps.
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        int numberOfSteps = steps.applyAsInt(rg, state);
        if (numberOfSteps<=0) {
            return new EffectStep<>(new LinkedList<>(), nextController);
        } else {
            return new EffectStep<>(new LinkedList<>(), new StepController((rg2, ds) -> numberOfSteps-1, nextController));
        }
    }

}
