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
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;

public class JSpearEnvironmentConditionalUpdateFunction extends JSpearAbstractEnvironmentFunction {


    private final JSpearExpressionEvaluationFunction guard;
    private final JSpearEnvironmentUpdateFunction thenFunction;
    private final JSpearEnvironmentUpdateFunction elseFunction;

    public JSpearEnvironmentConditionalUpdateFunction(JSpearVariableAllocation allocation, JSpearExpressionEvaluationFunction guard, JSpearEnvironmentUpdateFunction thenFunction, JSpearEnvironmentUpdateFunction elseFunction) {
        super(allocation);
        this.guard = guard;
        this.thenFunction = thenFunction;
        this.elseFunction = elseFunction;
    }

    public JSpearEnvironmentConditionalUpdateFunction(JSpearVariableAllocation allocation, JSpearExpressionEvaluationFunction guard, JSpearEnvironmentUpdateFunction thenFunction) {
        this(allocation, guard, thenFunction, null);

    }

    @Override
    public List<DataStateUpdate> apply(RandomGenerator randomGenerator, JSpearStore jSpearStore) {
        if (JSpearValue.isTrue(this.guard.eval(randomGenerator, jSpearStore))) {
            return thenFunction.apply(randomGenerator, jSpearStore);
        } else {
            if (elseFunction != null) {
                return elseFunction.apply(randomGenerator, jSpearStore);
            } else {
                return List.of();
            }
        }
    }
}
