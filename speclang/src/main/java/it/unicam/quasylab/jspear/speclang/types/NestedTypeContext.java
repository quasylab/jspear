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

public class NestedTypeContext implements TypeEvaluationContext {
    private final TypeEvaluationContext innerContext;
    private final TypeEvaluationContext outerContext;

    public NestedTypeContext(TypeEvaluationContext innerContext, TypeEvaluationContext outerContext) {
        this.innerContext = innerContext;
        this.outerContext = outerContext;
    }

    @Override
    public boolean isDefined(String name) {
        return this.innerContext.isDefined(name)||this.outerContext.isDefined(name);
    }

    @Override
    public boolean isAReference(String name) {
        if (this.innerContext.isDefined(name)) {
            return this.innerContext.isAReference(name);
        } else {
            return this.outerContext.isAReference(name);
        }
    }

    @Override
    public JSpearType getTypeOf(String name) {
        if (this.innerContext.isDefined(name)) {
            return this.innerContext.getTypeOf(name);
        } else {
            return this.outerContext.getTypeOf(name);
        }
    }

    @Override
    public boolean isAFunction(String functionName) {
        if (this.innerContext.isDefined(functionName)) {
            return this.innerContext.isAFunction(functionName);
        } else {
            return this.outerContext.isAFunction(functionName);
        }
    }

    @Override
    public JSpearType[] getArgumentsType(String functionName) {
        if (this.innerContext.isDefined(functionName)) {
            return this.innerContext.getArgumentsType(functionName);
        } else {
            return this.outerContext.getArgumentsType(functionName);
        }
    }

    @Override
    public JSpearType getReturnType(String functionName) {
        if (this.innerContext.isDefined(functionName)) {
            return this.innerContext.getReturnType(functionName);
        } else {
            return this.outerContext.getReturnType(functionName);
        }
    }
}
