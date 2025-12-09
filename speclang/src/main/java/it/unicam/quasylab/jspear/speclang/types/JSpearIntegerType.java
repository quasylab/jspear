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

import it.unicam.quasylab.jspear.ds.DataRange;
import it.unicam.quasylab.jspear.speclang.values.JSPearInteger;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

/**
 * A class representing an integer type.
 */
public final class JSpearIntegerType implements JSpearType {

    private static JSpearIntegerType instance;

    public static JSpearIntegerType getInstance() {
        if (instance == null) {
            instance = new JSpearIntegerType();
        }
        return instance;
    }

    public JSpearIntegerType() {}

    @Override
    public JSpearType merge(JSpearType other) {
        if ((this == other)) return this;
        if ((other.deterministicType()==REAL_TYPE)||(other.deterministicType()==INTEGER_TYPE)) return other;
        return JSpearType.ERROR_TYPE;
    }

    @Override
    public boolean isCompatibleWith(JSpearType other) {
        return (other.deterministicType() == INTEGER_TYPE);
    }

    @Override
    public boolean isNumerical() {
        return true;
    }


    @Override
    public boolean isError() {
        return false;
    }

    @Override
    public boolean canBeMergedWith(JSpearType other) {
        JSpearType deterministicOther = other.deterministicType();
        return (this==deterministicOther)||(deterministicOther==REAL_TYPE);
    }

    @Override
    public boolean isInteger() {
        return true;
    }

    @Override
    public JSpearValue valueOf(double v) {
        return new JSPearInteger((int) v);
    }

    @Override
    public DataRange getDefaultDataRange() {
        return new DataRange(Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    @Override
    public String toString() {
        return JSpearType.INTEGER_TYPE_STRING;
    }

    public JSpearValue fromDouble(double v) {
        return new JSPearInteger((int) v);
    }
}
