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

/**
 * Class ProbabilisticInterleavingController implements a controller consisting of
 * two controllers running in probabilistic interleaving.
 * At each step the effects and the transitions of the controller are chosen probabilistically
 * between the ones of the two controllers.
 */
public class ProbabilisticInterleavingController implements Controller {

    private final double p;
    private final Controller leftController;
    private final Controller rightController;

    /**
     * Creates a new controller consisting of the probabilistic interleaving of the two given controllers.
     *
     * @param p a probability weight.
     * @param leftController a controller.
     * @param rightController a controller.
     */
    public ProbabilisticInterleavingController(double p, Controller leftController, Controller rightController) {
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
     * The controller that takes a transition to the probabilistic interleaving
     * of the next behaviour of the controller that applied the effect,
     * and the current behaviour of the other controller.
     *
     * @param rg random generator
     * @param state the current data state
     * @return with probability <code>p</code>, the effect of <code>leftController</code> on <code>state</code>
     * and the probabilistic interleaving of the target of the transition of <code>leftController</code> with <code>rightController</code>;
     * with probability <code>1-p</code>, the effect of <code>rightController</code> on <code>state</code>
     * and the probabilistic interleaving of <code>leftController</code> with the target of the transition of <code>rightController</code>.
     */
    @Override
    public EffectStep<Controller> next(RandomGenerator rg, DataState state) {
        double p = rg.nextDouble();
        if (p <= this.p) {
            EffectStep<Controller> effectStep = this.leftController.next(rg, state);
            List<DataStateUpdate> updates = effectStep.effect();
            ProbabilisticInterleavingController c = new ProbabilisticInterleavingController(this.p, effectStep.next(), this.rightController);
            return new EffectStep<>(updates, c);
        }
        else{
            EffectStep<Controller> effectStep = this.rightController.next(rg, state);
            List<DataStateUpdate> updates = effectStep.effect();
            ProbabilisticInterleavingController c = new ProbabilisticInterleavingController(this.p, this.leftController,effectStep.next());
            return new EffectStep<>(updates, c);
        }
    }
}
