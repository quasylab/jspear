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
import it.unicam.quasylab.jspear.speclang.variables.JSpearStore;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;

public class JSpearEnvironmentAssignmentFunction extends JSpearAbstractEnvironmentFunction {

    private final BiFunction<RandomGenerator, JSpearStore, Optional<DataStateUpdate>> assignments;

    public JSpearEnvironmentAssignmentFunction(JSpearVariableAllocation allocation, BiFunction<RandomGenerator, JSpearStore, Optional<DataStateUpdate>> assignments) {
        super(allocation);
        this.assignments = assignments;
    }

    @Override
    public List<DataStateUpdate> apply(RandomGenerator randomGenerator, JSpearStore jSpearStore) {
        Optional<DataStateUpdate> optionalUpdate = assignments.apply(randomGenerator, jSpearStore);
        return optionalUpdate.map(List::of).orElseGet(List::of);
    }
}
