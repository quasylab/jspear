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

package it.unicam.quasylab.jspear.ds;

/**
 * Defines classic order relations over reals
 * used in the specification of distance expressions and formulae.
 */
public enum RelationOperator {

    LESS_THAN,
    LESS_OR_EQUAL_THAN,
    EQUAL_TO,
    GREATER_THAN,
    GREATER_OR_EQUAL_THAN;

    /**
     * Returns the Boolean evaluation of each relation operator between two given reals.
     *
     * @param v1 a real number
     * @param v2 a real number
     * @return the Boolean evaluation of the order relations between <code>v1</code> and <code>v2</code>
     */
    public boolean eval(double v1, double v2) {
        return switch (this) {
            case LESS_THAN -> v1<v2;
            case LESS_OR_EQUAL_THAN -> v1<=v2;
            case EQUAL_TO ->  v1 == v2;
            case GREATER_OR_EQUAL_THAN -> v1>=v2;
            case GREATER_THAN -> v1>v2;
        };
    }

    /**
     * Returns the relation operator corresponding to the given string.
     *
     * @param op a string representing an order relation over reals.
     * @return the relation operator corresponding to <code>op</code>.
     * @throws IllegalArgumentException if the given string does not correspond to any relation operator.
     */
    public static RelationOperator get(String op) {
        if (">".equals(op)) {
            return GREATER_THAN;
        }
        if (">=".equals(op)) {
            return GREATER_OR_EQUAL_THAN;
        }
        if ("==".equals(op)) {
            return EQUAL_TO;
        }
        if ("<".equals(op)) {
            return LESS_THAN;
        }
        if ("<=".equals(op)) {
            return LESS_OR_EQUAL_THAN;
        }
        throw new IllegalArgumentException("Unknown relation "+op);
    }

}
