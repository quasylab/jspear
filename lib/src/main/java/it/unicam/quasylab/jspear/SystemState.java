/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * This interface is implemented to define different models.
 */
public interface SystemState {

    /**
     * Returns the data state associated with this state.
     *
     * @return the data state associated with this state.
     */
    DataState getDataState();

    /**
     * Returns one state sampled among the one reachable from this state in one step.
     *
     * @param rg random generator used sample random expression.
     * @return one state sampled among the one reachable from this state in one step.
     */
    SystemState sampleNext(RandomGenerator rg);

    /**
     * Returns the system state where the data state is replaced with the given one.
     *
     * @param dataState new data state.
     * @return the system state where the data state is replaced with the given one.
     */
    SystemState setDataState(DataState dataState);


    /**
     * Returns the system state obtained by sampling one
     *
     * @param rg
     * @param function
     * @return
     */
    default SystemState apply(RandomGenerator rg, DataStateFunction function) {
        return this.setDataState(function.apply(rg, this.getDataState()));
    }

}
