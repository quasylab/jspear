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

package it.unicam.quasylab.jspear.ds;

import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.function.BiFunction;

/**
 * Instances of this interface are used to represent a random function from data states to data states.
 */
@FunctionalInterface
public interface DataStateFunction {

    /**
     * The <code>TICK_FUNCTION</code> is defined as a data state update that applies no modification to this data state.
     */
    BiFunction<RandomGenerator, DataState, List<DataStateUpdate>>  TICK_FUNCTION = (rg, ds) -> List.of();

    /**
     * Given a random generator, used to evaluate random expressions, and a data state,
     * samples an outcome of this expression.
     *
     * @param rg random generator used to evaluate random expressions.
     * @return a data state sampled among the ones reachable in one step from ds.
     */
    DataState apply(RandomGenerator rg, DataState ds);

    /**
     * Returns the composition of this function with a given one.
     *
     * @param other function that we want to apply immediately after this.
     * @return the outcome of the application of this function followed by the application of <code>other</code>.
     */
    default DataStateFunction compose(DataStateFunction other) {
        return (rg, ds) -> other.apply(rg, this.apply(rg, ds));
    }



}
