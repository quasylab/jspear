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
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.function.BiFunction;

/**
 * Class ActionController implements a controller that
 * executes a given action on the data state
 * and then evolves to another controller.
 */
public class ActionController implements Controller {

    private final BiFunction<RandomGenerator, DataState, List<DataStateUpdate>> action;
    private final Controller nextController;

    /**
     * Generates a controller that
     * executes the given <code>action</code>
     * and then behaves like <code>nextController</code>.
     *
     * @param action effect on data state.
     * @param nextController controller enabled after the execution of the action
     */
    public ActionController(BiFunction<RandomGenerator, DataState, List<DataStateUpdate>> action, Controller nextController) {
        this.action = action;
        this.nextController = nextController;
    }

    /**
     * The effect of the action is applied to the current data state,
     * and the controller at the next step is modelled by <code>nextController</code>.
     *
     * @param rg random generator
     * @param state the current data state
     * @return the pair consisting of the effect of the action on <code>state</code>,
     * and the controller behaviour at the next step <code>nextController</code>.
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        return new EffectStep<>(action.apply(rg, state), nextController);
    }
}
