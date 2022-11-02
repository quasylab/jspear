/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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
public class JSpearNameResolver {

    private final Map<String, Variable> variables = new HashMap<>();

    /**
     * Returns the variable with the given name. A <code>null</code> value is returned if not variable with the
     * given name does exist.
     *
     * @param name variable name.
     * @return the variable with the given name.
     */
    public Variable get(String name) {
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
    public Variable getOrRegister(String name) {
        return variables.computeIfAbsent(name, this::getVariable);
    }


    private Variable getVariable(String name) {
        return new Variable(name, variables.size());
    }

    /**
     * Returmns the number of declared variables.
     *
     * @return the number of declared variables.
     */
    public int size() {
        return this.variables.size();
    }

}
