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
import it.unicam.quasylab.jspear.speclang.values.JSpearReal;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

/**
 * A class representing a real type.
 */
public final class JSpearRealType implements JSpearType {

    private static JSpearRealType instance;

    public static JSpearRealType getInstance() {
        if (instance == null) {
            instance = new JSpearRealType();
        }
        return instance;
    }

    @Override
    public JSpearType merge(JSpearType other) {
        if ((this == other)) return this;
        if (other.deterministicType()==REAL_TYPE) return other;
        if (other.deterministicType()==INTEGER_TYPE) {
            if (other.isRandom()) {
                return new JSpearRandomType(this);
            } else {
                return this;
            }
        }
        return JSpearType.ERROR_TYPE;
    }

    @Override
    public boolean isCompatibleWith(JSpearType other) {
        return other.isNumerical();
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
        return (this==deterministicOther)||(deterministicOther==INTEGER_TYPE);
    }

    @Override
    public boolean isReal() {
        return true;
    }

    @Override
    public JSpearValue valueOf(double v) {
        return new JSpearReal(v);
    }

    @Override
    public DataRange getDefaultDataRange() {
        return new DataRange();
    }

    @Override
    public String toString() {
        return JSpearType.REAL_TYPE_STRING;
    }

}
