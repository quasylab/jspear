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

package it.unicam.quasylab.jspear.speclang.semantics;

import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.JSpearStore;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariable;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;

public class JSpearEnvironmentLetUpdateFunction extends JSpearAbstractEnvironmentFunction implements BiFunction<RandomGenerator, JSpearStore, List<DataStateUpdate>>, JSpearEnvironmentUpdateFunction {

    private final JSpearVariable[] localVariables;
    private final JSpearExpressionEvaluationFunction[] localVariablesValues;

    private final JSpearEnvironmentUpdateFunction updates;

    public JSpearEnvironmentLetUpdateFunction(JSpearVariableAllocation allocation, JSpearVariable[] localVariables, JSpearExpressionEvaluationFunction[] localVariablesValues, JSpearEnvironmentUpdateFunction updates) {
        super(allocation);
        this.localVariables = localVariables;
        this.localVariablesValues = localVariablesValues;
        this.updates = updates;
    }

    @Override
    public List<DataStateUpdate> apply(RandomGenerator randomGenerator, JSpearStore jSpearStore) {
        Map<JSpearVariable, JSpearValue> localStore = new HashMap<>();
        for (int i = 0; i < localVariables.length; i++) {
            localStore.put(localVariables[i], localVariablesValues[i].eval(randomGenerator, JSpearStore.storeOf(localStore, jSpearStore)));
        }
        return updates.apply(randomGenerator, JSpearStore.storeOf(localStore, jSpearStore));
    }

}
