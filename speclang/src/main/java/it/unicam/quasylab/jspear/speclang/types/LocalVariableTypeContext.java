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

package it.unicam.quasylab.jspear.speclang.types;

public class LocalVariableTypeContext implements TypeEvaluationContext {
    private final String name;
    private final JSpearType type;

    public LocalVariableTypeContext(String name, JSpearType type) {
        this.name = name;
        this.type = type;
    }

    @Override
    public boolean isDefined(String name) {
        return this.name.equals(name);
    }

    @Override
    public boolean isAReference(String name) {
        return this.name.equals(name);
    }

    @Override
    public JSpearType getTypeOf(String name) {
        return (this.name.equals(name)?this.type:null);
    }

    @Override
    public boolean isAFunction(String functionName) {
        return false;
    }

    @Override
    public JSpearType[] getArgumentsType(String functionName) {
        return null;
    }

    @Override
    public JSpearType getReturnType(String functionName) {
        return null;
    }
}
