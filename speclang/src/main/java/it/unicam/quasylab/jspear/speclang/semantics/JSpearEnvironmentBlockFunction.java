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

/**
 * Instances of this class act as container for a list of environment functions.
 */
public class JSpearEnvironmentBlockFunction extends JSpearAbstractEnvironmentFunction {

    private final List<JSpearEnvironmentUpdateFunction> commands;

    /**
     * Creates a new block funciton containing the given list of functions.
     *
     * @param allocation allocation used to solve variables
     * @param commands executed commands in the block
     */
    public JSpearEnvironmentBlockFunction(JSpearVariableAllocation allocation, List<JSpearEnvironmentUpdateFunction> commands) {
        super(allocation);
        this.commands = commands;
    }

    @Override
    public List<DataStateUpdate> apply(RandomGenerator randomGenerator, JSpearStore jSpearStore) {
        return commands.stream().flatMap(c -> c.apply(randomGenerator, jSpearStore).stream()).toList();
    }
}
