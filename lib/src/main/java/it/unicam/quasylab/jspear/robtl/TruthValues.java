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

package it.unicam.quasylab.jspear.robtl;

/**
 * Defines the three values of the three-valued semantics,
 * TRUE, FALSE, and UNKNOWN,
 * and the truth tables of standard Boolean operators for them.
 */
public enum TruthValues {
    TRUE,
    FALSE,
    UNKNOWN;

    /**
     * Three-valued semantics of conjunction.
     *
     * @param value1 a truth value
     * @param value2 a truth value
     * @return the three-valued evaluation of <code>value1 & value2</code>.
     */
    public static TruthValues and(TruthValues value1, TruthValues value2) {
        if (value1 == TruthValues.FALSE || value2 == TruthValues.FALSE) return TruthValues.FALSE;
        if (value1 == TruthValues.TRUE && value2 == TruthValues.TRUE) return TruthValues.TRUE;
        return TruthValues.UNKNOWN;
    }

    /**
     * Three-valued semantics of disjunction.
     *
     * @param value1 a truth value
     * @param value2 a truth value
     * @return the three-valued evaluation of <code>value1 | value2</code>.
     */
    public static TruthValues or(TruthValues value1, TruthValues value2) {
        if (value1 == TruthValues.TRUE || value2 == TruthValues.TRUE) return TruthValues.TRUE;
        if (value1 == TruthValues.FALSE && value2 == TruthValues.FALSE) return TruthValues.FALSE;
        return TruthValues.UNKNOWN;
    }

    /**
     * Three-valued semantics of negation.
     *
     * @param value a truth value
     * @return the three-valued evaluation of <code>!value</code>
     */
    public static TruthValues neg(TruthValues value) {
        if (value == TruthValues.UNKNOWN) return TruthValues.UNKNOWN;
        if (value == TruthValues.FALSE) return TruthValues.TRUE;
        return TruthValues.FALSE;
    }

    /**
     * Three-valued semantics of implication.
     *
     * @param value1 a truth value
     * @param value2 a truth value
     * @return the three-valued evaluation of <code>!value1 | value2</code>
     */
    public static TruthValues imply(TruthValues value1, TruthValues value2) {
        return TruthValues.or(TruthValues.neg(value1), value2);
    }

    /**
     * Auxiliary method used to assign a real number to the three truth values.
     *
     * @return 1 for TRUE, 0 for UNKNOWN, and -1 for FALSE.
     */
    public double valueOf() {
        return switch (this) {
            case TRUE -> 1.0;
            case UNKNOWN -> 0.0;
            case FALSE -> -1.0;
        };
    }
}