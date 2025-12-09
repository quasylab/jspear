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
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

/**
 * This type describes error values. This class is a singleton.
 */
public final class JSpearErrorType implements JSpearType {

    private static JSpearErrorType instance;

    /**
     * Returns the instance of error type.
     *
     * @return the instance of error type.
     */
    public static JSpearErrorType getInstance() {
        if (instance == null) {
            instance = new JSpearErrorType();
        }
        return instance;
    }

    /**
     * Creates a new instance of error type.
     */
    private JSpearErrorType() {}

    @Override
    public JSpearType merge(JSpearType other) {
        return this;
    }

    @Override
    public boolean isCompatibleWith(JSpearType other) {
        return false;
    }

    @Override
    public boolean isNumerical() {
        return false;
    }


    @Override
    public boolean isError() {
        return true;
    }

    @Override
    public boolean canBeMergedWith(JSpearType other) {
        return true;
    }

    @Override
    public JSpearValue valueOf(double v) {
        return JSpearValue.ERROR_VALUE;
    }

    @Override
    public DataRange getDefaultDataRange() {
        return new DataRange(Double.NaN, Double.NaN);
    }

    @Override
    public String toString() {
        return JSpearType.ERROR_TYPE_STRING;
    }

}
