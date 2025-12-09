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

import java.util.function.BiPredicate;

/**
 * Class IfThenElseController implements a controller that
 * implements an if-then-else behaviour.
 */
public class IfThenElseController implements Controller{

    private final BiPredicate<RandomGenerator, DataState> guard;
    private final Controller thenController;
    private final Controller elseController;

    /**
     * Creates a controller that
     * behaves like <code>thenController</code> when the <code>guard</code> is satisfied,
     * and like <code>elseController</code> when this is not satisfied.
     *
     * @param guard a predicate on data states.
     * @param thenController the behaviour when the guard is satisfied.
     * @param elseController the behaviour when the guard is not satisfied.
     */
    public IfThenElseController(BiPredicate<RandomGenerator, DataState> guard, Controller thenController, Controller elseController) {
        this.guard = guard;
        this.thenController = thenController;
        this.elseController = elseController;
    }

    /**
     * Defines the effect of an IfThenElseController:
     * if the <code>guard</code> is satisfied,
     * the effect of <code>thenController</code> is applied,
     * otherwise, the effect of <code>elseController</code> is applied.
     *
     * @param rg random generator
     * @param state the current data state
     * @return the effect of <code>thenController</code> on <code>state</code> if the <code>guard</code> evaluates to <code>true</code>,
     * the effect of <code>elseController</code> on <code>state</code> otherwise.
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        return (guard.test(rg, state)?thenController.next(rg, state):elseController.next(rg, state));
    }


}
