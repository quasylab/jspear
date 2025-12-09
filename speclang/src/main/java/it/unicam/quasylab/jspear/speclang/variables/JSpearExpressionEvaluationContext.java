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

import it.unicam.quasylab.jspear.speclang.semantics.JSpearFunction;
import it.unicam.quasylab.jspear.speclang.types.JSpearCustomType;
import it.unicam.quasylab.jspear.speclang.values.JSpearCustomValue;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class JSpearExpressionEvaluationContext {

    private final Map<String, JSpearValue> values;

    private final Map<String, JSpearFunction> functions;

    private final Map<String, JSpearCustomValue> customValues;

    public JSpearExpressionEvaluationContext(Map<String, JSpearValue> parameters) {
        values = new HashMap<>();
        functions = new HashMap<>();
        customValues = new HashMap<>();
        values.putAll(parameters);
    }


    public void set(String name, JSpearValue value) {
        this.values.put(name, value);
    }

    public boolean isDefined(String name) {
        return values.containsKey(name)||customValues.containsKey(name);
    }

    public JSpearValue get(String name) {
        if (this.values.containsKey(name)) {
            return this.values.get(name);
        }
        if (this.customValues.containsKey(name)) {
            return this.values.get(name);
        }
        return JSpearValue.ERROR_VALUE;
    }

    public boolean isAFunction(String name) {
        return this.functions.containsKey(name);
    }

    public JSpearFunction getFunction(String name) {
        return this.functions.get(name);
    }

    public void recordFunction(String name, JSpearFunction function) {
        this.functions.put(name, function);
    }

    public void recordType(String typeName, JSpearCustomType type) {
        JSpearCustomValue[] values = type.getValues();
        Arrays.stream(values).sequential().forEach(v -> this.customValues.put(v.name(), v));
    }
}
