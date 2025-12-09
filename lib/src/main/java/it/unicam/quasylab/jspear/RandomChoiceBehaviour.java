/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
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

package it.unicam.quasylab.jspear;

import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.EffectStep;
import it.unicam.quasylab.jspear.ds.DataState;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * This is a controller that randomly selects one among two possible behaviours.
 */
public class RandomChoiceBehaviour implements Controller {

    private final Controller leftController;
    private final double prob;
    private final Controller rightController;

    /**
     * Creates a controller that with probability <code>prob</code> works like <code>leftController</code> and
     * with probability <code>1-prob</code> like <code>rightController</code>.
     * @param leftController the controller that will be selected with probability <code>prob</code>.
     * @param prob a value between 0 and 1.
     * @param rightController the controller that will be selected with probability <code>1-prob</code>.
     */
    public RandomChoiceBehaviour(Controller leftController, double prob, Controller rightController) {
        this.leftController = leftController;
        this.prob = prob;
        this.rightController = rightController;
    }

    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        return null;
    }
}
