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

import java.util.HashMap;
import java.util.Map;

/**
 * This class is used to associate names to variables.
 */
public class JSpearVariableRegistry {

    private final Map<String, JSpearVariable> variables = new HashMap<>();

    /**
     * Returns the variable with the given name. A <code>null</code> value is returned if not variable with the
     * given name does exist.
     *
     * @param name variable name.
     * @return the variable with the given name.
     */
    public JSpearVariable get(String name) {
        return variables.get(name);
    }

    /**
     * Returns true if a variable with the given name does exist.
     *
     * @param name variable name
     * @return true if a variable with the given name does exist.
     */
    public boolean isDeclared(String name) {
        return variables.containsKey(name);
    }

    /**
     * Returns the variable with the given name. If it does not exist a new variable is allocated.
     *
     * @param name variable name.
     * @return the variable with the given name.
     */
    public JSpearVariable getOrRegister(String name) {
        return variables.computeIfAbsent(name, this::getVariable);
    }


    private JSpearVariable getVariable(String name) {
        return new JSpearVariable(name, variables.size());
    }

    /**
     * Returmns the number of declared variables.
     *
     * @return the number of declared variables.
     */
    public int size() {
        return this.variables.size();
    }

    public void record(String name) {
        getOrRegister(name);
    }
}
