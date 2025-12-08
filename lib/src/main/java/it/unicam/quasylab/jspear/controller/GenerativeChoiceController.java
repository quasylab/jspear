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

package it.unicam.quasylab.jspear.controller;

import it.unicam.quasylab.jspear.ds.DataState;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.function.BiPredicate;

/**
 * Class GenerativeChoiceController implements a controller that
 * implements a generative probabilistic choice between two controllers.
 */
public class GenerativeChoiceController implements Controller{

    private final double p;
    private final Controller leftController;
    private final Controller rightController;

    /**
     * Creates a controller that
     * behaves like <code>leftController</code> with probability <code>p</code>,
     * and like <code>rightController</code> with probability <code>1-p</code>.
     *
     * @param p a probability weight.
     * @param leftController the controller chosen with probability <code>p</code>.
     * @param rightController the controller chosen with probability <code>1-p</code>.
     */
    public GenerativeChoiceController(double p, Controller leftController, Controller rightController) {
        if (p<0 || p>1) {
            throw new IllegalArgumentException("Illegal probability weight!");
        }
        this.p = p;
        this.leftController = leftController;
        this.rightController = rightController;
    }

    /**
     * With probability <code>p</code>, the effect of <code>leftController</code> is applied to the data state,
     * and with probability <code>1-p</code>, the one of <code>rightController</code> is applied.
     * The controller than takes a transition to the controller implementing the behaviour at the next step of the chosen one.
     *
     * @param rg random generator
     * @param state the current data state
     * @return the effect and transition of <code>leftController</code> on <code>state</code> with probability <code>p</code>,
     * and those of <code>rightController</code> with probability <code>1-p</code>
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        double p = rg.nextDouble();
        if (p <= this.p) {
            return leftController.next(rg,state);
        }
        else{
            return rightController.next(rg,state);
        }
    }

}
