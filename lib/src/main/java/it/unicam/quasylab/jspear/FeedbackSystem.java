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

package it.unicam.quasylab.jspear;

import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.EffectStep;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateBooleanExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.feedback.Feedback;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Represent a system in the evolution sequence model
 * in which the evolution is controlled by a given feedback.
 */
public class FeedbackSystem implements SystemState {

    private final Controller controller;
    private final DataStateFunction environment;
    private final DataState state;
    private final Feedback feedback;

    /**
     * Creates a system with the given agent, environment and data state.
     * @param controller process modelling the agent,
     * @param environment set of functions modelling the environment,
     * @param state current data state.
     */
    public FeedbackSystem(Controller controller, DataStateFunction environment, DataState state, Feedback feedback) {
        this.controller = controller;
        this.environment = environment;
        this.state = state;
        this.feedback = feedback;
    }

    @Override
    public DataState getDataState() {
        return state;
    }

    @Override
    //public SystemState sampleNext(RandomGenerator rg) {
    //    EffectStep<Controller> step = controller.next(rg, state);
    //    return new FeedbackSystem(step.next(), environment, environment.apply(rg, feedback.apply(rg,state.apply(step.effect()))), feedback.next());
    //}

    public SystemState sampleNext(RandomGenerator rg) {
        EffectStep<Controller> step = controller.next(rg, state);
        int c_step = state.getStep();
        DataState newState = environment.apply(rg, feedback.apply(rg,state.apply(step.effect())));
        newState.setStep(c_step +1);
        return new FeedbackSystem(step.next(), environment, newState, feedback.next());
    }

    @Override
    public SystemState setDataState(DataState dataState) {
        return new FeedbackSystem(controller, environment, dataState, feedback);
    }

    @Override
    public SystemState sampleNextCond(RandomGenerator rg, DataStateBooleanExpression condition) {
        SystemState result = this;
        DataState ds = this.state;
        while(!condition.eval(ds)) {
            result = result.sampleNext(rg);
            ds = result.getDataState();
        }
        return result;
    }

}

