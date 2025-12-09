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

package nl.tue.Monitoring;

import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateBooleanExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import org.apache.commons.math3.random.RandomGenerator;

public class PerceivedSystemState implements SystemState {

    private final DataState ds;

    public PerceivedSystemState(DataState ds) {
        this.ds = ds;
    }

    @Override
    public DataState getDataState() {
        return ds;
    }

    @Override
    public PerceivedSystemState setDataState(DataState dataState) {
       return new PerceivedSystemState(dataState);
    }

    @Override
    public SystemState sampleNextCond(RandomGenerator rg, DataStateBooleanExpression cond) {
        throw new UnsupportedOperationException("Future system state cannot be predicted from a perceived data state");
    }

    @Override
    public SystemState sampleNext(RandomGenerator rg) {
        throw new UnsupportedOperationException("Future system state cannot be predicted from a perceived data state");
    }


    /**
     * Returns the sampling of the given expression applied to this perceived system state.
     *
     * @param rg       random generator used in the sampling
     * @param function random expression to sample
     * @return the sampling of the given expression applied to this perceived system state.
     */
    @Override
    public PerceivedSystemState apply(RandomGenerator rg, DataStateFunction function) {
        return this.setDataState(function.apply(rg, this.getDataState()));
    }
}
