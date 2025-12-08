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

package it.unicam.quasylab.jspear.ds;


import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.function.BiFunction;

/**
 * We use this class to create the updates that will be applied to the data stored in this data state.
 */
public final class DataStateUpdate {

    private final int index;

    private final double value;

    /**
     * Creates an update for the datum at a given index with a given value.
     *
     * @param index position of the datum in this data state
     * @param value value that will be assigned to the datum once the update is applied to this data state.
     */
    public DataStateUpdate(int index, double value) {
        this.index = index;
        this.value = value;
    }

    /**
     * Returns a data state update, as a list with one element, with a given index and a given value.
     *
     * @param idx index of the datum to be updated
     * @param value value that will be assigned to the datum once the update is applied to this data state
     * @return a one element list containing the data state update for the datum at index <code>idx</code> with value <code>value</code>.
     */
    public static BiFunction<RandomGenerator, DataState, List<DataStateUpdate>> set(int idx, double value) {
        return (rg, ds) -> List.of(new DataStateUpdate(idx, value));
    }

    /**
     * Returns the data state update expressed as a string.
     *
     * @return the string <code>index<-value</code> corresponding to this data state update.
     */
    @Override
    public String toString() {
        return index+"<-"+value;
    }

    /**
     * Returns the index of this data state update.
     *
     * @return parameter <code>index</code>.
     */
    public int getIndex() {
        return index;
    }

    /**
     * Returns the value of this data state update.
     *
     * @return parameter <code>value</code>
     */
    public double getValue() {
        return value;
    }
}
