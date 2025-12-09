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
import it.unicam.quasylab.jspear.speclang.values.JSpearBoolean;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

/**
 * This type describes the set of boolean values.
 */
public final class JSpearBooleanType implements JSpearType {

    private static JSpearBooleanType instance;

    public static JSpearBooleanType getInstance() {
        if (instance == null) {
            instance = new JSpearBooleanType();
        }
        return instance;
    }

    private JSpearBooleanType() {}

    @Override
    public JSpearType merge(JSpearType other) {
        if (other == JSpearType.BOOLEAN_TYPE) {
            return this;
        }
        if (other.deterministicType() == JSpearType.BOOLEAN_TYPE) {
            return other;
        }
        return JSpearType.ERROR_TYPE;
    }

    @Override
    public boolean isCompatibleWith(JSpearType other) {
        return (other.deterministicType() == JSpearType.BOOLEAN_TYPE);
    }

    @Override
    public boolean isNumerical() {
        return false;
    }


    @Override
    public boolean isError() {
        return false;
    }

    @Override
    public boolean canBeMergedWith(JSpearType other) {
        return (other.deterministicType() == JSpearType.BOOLEAN_TYPE);
    }

    @Override
    public boolean isBoolean() {
        return true;
    }

    @Override
    public JSpearValue valueOf(double v) {
        if (v==0) {
            return JSpearBoolean.FALSE;
        } else {
            return JSpearBoolean.TRUE;
        }
    }

    @Override
    public DataRange getDefaultDataRange() {
        return new DataRange(0, 1.0);
    }

    @Override
    public String toString() {
        return JSpearType.BOOLEAN_TYPE_STRING;
    }

    public JSpearValue fromDouble(double v) {
        return (v>0.0?JSpearBoolean.TRUE:JSpearBoolean.FALSE);
    }
}
