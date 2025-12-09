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

package it.unicam.quasylab.jspear.speclang.variables;

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * This interface represents the store used to evaluate expressions. Each store associates variables with values.
 */
public interface JSpearStore {

    static JSpearStore storeOf(Map<JSpearVariable, JSpearValue> localStore, JSpearStore jSpearStore) {
        return v -> {
            if (localStore.containsKey(v)) {
                return localStore.get(v);
            } else {
                return jSpearStore.get(v);
            }
        };
    }

    static JSpearStore storeOf(JSpearVariable[] localVariables, JSpearValue[] args) {
        Map<JSpearVariable, JSpearValue> localStore = new HashMap<>();
        for(int i=0; i<localVariables.length; i++) {
            localStore.put(localVariables[i], args[i]);
        }
        return JSpearStore.storeOf(localStore);
    }

    /**
     * Returns the value associated with the given variable.
     *
     * @param variable variable to read.
     * @return the value associated with the given element index.
     */
    JSpearValue get(JSpearVariable variable);


    /**
     * Returns a new store that enriches the given one with the binging of <code>variable</code> to <code>value</code>.
     *
     * @param variable assigned variables
     * @param value assigned value
     * @param store enriched value
     * @return a new store that enriches the given one with the binging of <code>variable</code> to <code>value</code>.
     */
    static JSpearStore let(JSpearVariable variable, JSpearValue value, JSpearStore store) {
        return v -> (variable.equals(v)?value:store.get(variable));
    }

    /**
     * Returns the store whose binding are the same as the given map.
     *
     * @param map a map associating variables to values.
     * @return the store whose binding are the same as the given map.
     */
    static JSpearStore storeOf(Map<JSpearVariable, JSpearValue> map) {
        return v -> map.getOrDefault(v, JSpearValue.ERROR_VALUE);
    }

    /**
     * Returns the store whose binding are defined in terms of the given allocation and data state.
     *
     * @param allocation variable allocation.
     * @param state data state.
     * @return the store whose binding are defined in terms of the given allocation and data state.
     */
    static JSpearStore storeOf(JSpearVariableAllocation allocation, DataState state) {
        return v -> allocation.get(v, state);
    }
}


