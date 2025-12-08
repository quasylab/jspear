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
import it.unicam.quasylab.jspear.ds.DataStateBooleanExpression;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Represents a system controlled by controller.
 */
public class TimedSystem implements SystemState {

    private final Controller controller;
    private final DataStateFunction environment;
    private final DataState state;
    private final DataStateExpression generateNextTime;

    /**
     * Creates a system with the given controller and the given state.
     *  @param controller system controller.
     * @param environment environment
     * @param state current data state.
     */
    public TimedSystem(Controller controller, DataStateFunction environment, DataState state, DataStateExpression generateNextTime) {
        this.controller = controller;
        this.environment = environment;
        this.state = state;
        this.generateNextTime = generateNextTime;
    }

    @Override
    public DataState getDataState() {
        return state;
    }



    public TimedSystem sampleNextMicro(RandomGenerator rg){
        EffectStep<Controller> step = controller.next(rg, state);
        return new TimedSystem(step.next(), environment, environment.apply(rg, state.apply(step.effect())),generateNextTime);
    }

    @Override
    public SystemState sampleNext(RandomGenerator rg) {
        TimedSystem next = new TimedSystem(this.controller,this.environment,this.state,this.generateNextTime);
        DataState ds = next.state;
        double t = next.generateNextTime.eval(ds);
        double sum_t = t + ds.getTimeReal();
        while(sum_t < ds.getTimeStep() + ds.getGranularity()){
            next = next.sampleNextMicro(rg);
            ds = next.getDataState();
            ds.setTimeDelta(t);
            ds.setTimeReal(t + ds.getTimeReal());
            t = next.generateNextTime.eval(ds);
            sum_t = sum_t + t;
        }
        next = next.sampleNextMicro(rg);
        ds = next.getDataState();
        ds.setTimeDelta(t);
        ds.setTimeReal(t + ds.getTimeReal());
        ds.setTimeStep(ds.getTimeStep() + ds.getGranularity());
        return next;
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

    @Override
    public SystemState setDataState(DataState dataState) {
        return new TimedSystem(controller, environment, dataState, generateNextTime);
    }


}
