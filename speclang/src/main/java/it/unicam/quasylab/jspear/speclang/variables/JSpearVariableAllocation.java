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

import it.unicam.quasylab.jspear.ds.DataRange;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.speclang.types.JSpearType;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * This class is used to allocate variables in data store.
 */
public class JSpearVariableAllocation {

    private final Map<JSpearVariable, JSpearVariableAllocationData> allocationData = new HashMap<>();


    /**
     * Adds the given variable with the given type to this allocation.
     *
     * @param variable declared variable.
     * @param type variable type.
     */
    public void add(JSpearVariable variable, JSpearType type) {
        if (allocationData.containsKey(variable)) {
            throw new IllegalArgumentException("Duplicated variable "+variable);
        }
        this.allocationData.put(variable, new JSpearVariableAllocationData(variable, allocationData.size(), type));
    }

    public void add(JSpearVariable variable, JSpearType type, DataRange range) {
        if (allocationData.containsKey(variable)) {
            throw new IllegalArgumentException("Duplicated variable "+variable);
        }
        this.allocationData.put(variable, new JSpearVariableAllocationData(variable, allocationData.size(), type, range));
    }

    public JSpearValue get(JSpearVariable variable, DataState state) {
        JSpearVariableAllocationData variableAllocationData = allocationData.get(variable);
        if (variableAllocationData == null) {
            return JSpearValue.ERROR_VALUE;
        } else {
            return variableAllocationData.get(state);
        }
    }

    public Optional<DataStateUpdate> set(JSpearVariable variable, JSpearValue value) {
        JSpearVariableAllocationData variableAllocationData = allocationData.get(variable);
        if (variableAllocationData == null) {
            return Optional.empty();
        } else {
            return Optional.of(variableAllocationData.set(value));
        }
    }

    public DataState getDataState(Map<JSpearVariable, JSpearValue> initialValues) {
        DataRange[] range = new DataRange[this.allocationData.size()];
        double[] values = new double[this.allocationData.size()];
        for (JSpearVariableAllocationData v: this.allocationData.values()) {
            range[v.index] = v.range;
            values[v.index] = initialValues.get(v.variable).toDouble();
        }
        return new DataState(range, values);
    }

    private static class JSpearVariableAllocationData {

        private final JSpearVariable variable;

        private final int index;

        private final JSpearType type;

        private final DataRange range;

        private JSpearVariableAllocationData(JSpearVariable variable, int index, JSpearType type) {
            this(variable, index, type, type.getDefaultDataRange());
        }

        public JSpearVariableAllocationData(JSpearVariable variable, int index, JSpearType type, DataRange range) {
            this.variable = variable;
            this.index = index;
            this.type = type;
            this.range = range;
        }


        /**
         * Returns the value associated to the variable allocated with this object on the given state.
         *
         * @param state data state used to read variable value.
         * @return the value associated to the variable allocated with this object on the given state.
         */
        public JSpearValue get(DataState state) {
            return type.valueOf(state.get(index));
        }

        public DataStateUpdate set(JSpearValue value) {
            return new DataStateUpdate(index, value.toDouble());
        }
    }
}
