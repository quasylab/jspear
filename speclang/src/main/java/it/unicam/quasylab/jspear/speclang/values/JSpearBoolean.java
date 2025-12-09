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

package it.unicam.quasylab.jspear.speclang.values;

import it.unicam.quasylab.jspear.speclang.types.JSpearType;

import java.util.Objects;

public final class JSpearBoolean implements JSpearValue {

    public static final JSpearValue TRUE = new JSpearBoolean(true);
    public static final JSpearValue FALSE = new JSpearBoolean(false);
    private final boolean value;

    private JSpearBoolean(boolean value) {
        this.value = value;
    }

    public static JSpearValue of(boolean b) {
        return (b?TRUE:FALSE);
    }


    @Override
    public JSpearType getJSpearType() {
        return JSpearType.BOOLEAN_TYPE;
    }

    @Override
    public double toDouble() {
        return (value?1.0:0.0);
    }

    public  JSpearValue negate() {
        return (this.value?FALSE:TRUE);
    }

    public JSpearValue and(JSpearValue other) {
        if (other instanceof JSpearBoolean booleanValue) {
            return JSpearBoolean.of(this.value()&&booleanValue.value());
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue or(JSpearValue other) {
        if (other instanceof JSpearBoolean booleanValue) {
            return JSpearBoolean.of(this.value()||booleanValue.value());
        }
        return JSpearValue.ERROR_VALUE;
    }

    public boolean value() {
        return value;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        JSpearBoolean that = (JSpearBoolean) o;
        return value == that.value;
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }
}
