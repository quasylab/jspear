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
import it.unicam.quasylab.jspear.speclang.parsing.ParseErrorCollector;
import it.unicam.quasylab.jspear.speclang.parsing.ParseUtil;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

/**
 * This interface is used to model data types occurring in a JSpear specification.
 */
public sealed interface JSpearType permits JSpearBooleanType, JSpearCustomType, JSpearErrorType, JSpearIntegerType, JSpearRandomType, JSpearRealType {


    /**
     * Type assigned to expressions with errors.
     */
    JSpearType ERROR_TYPE = JSpearErrorType.getInstance();

    /**
     * Type assigned to boolean expressions.
     */
    JSpearType BOOLEAN_TYPE = JSpearBooleanType.getInstance();

    /**
     * Type assigned to integer expressions.
     */
    JSpearType INTEGER_TYPE = JSpearIntegerType.getInstance();

    /**
     * Type assigned to real expressions.
     */
    JSpearType REAL_TYPE = JSpearRealType.getInstance();



    /**
     * Type assigned to array expressions.
     */
    String INTEGER_TYPE_STRING = "int";
    String REAL_TYPE_STRING = "real";
    String BOOLEAN_TYPE_STRING = "bool";
    String ERROR_TYPE_STRING = "error";
    String RANDOM_TYPE_STRING = "random";

    /**
     * Returns the type obtained by merging <code>this</code> type with the <code>other</code>. An error type
     * is returned if the two types are not compatible.
     *
     * @param other another type.
     * @return the type obtained by merging <code>this</code> type with the <code>other</code>.
     */
    JSpearType merge(JSpearType other);

    /**
     * Returns the type obtained by merging <code>one</code> type with the <code>other</code>. An error type
     *  is returned if the two types are not compatible.
     * @param one a type.
     * @param other another type.
     * @return the type obtained by merging <code>one</code> type with the <code>other</code>.
     */
    static JSpearType merge(JSpearType one, JSpearType other) {
        return one.merge(other);
    }

    /**
     * This method returns true if <code>this</code> type is compatible with
     * the <code>other</code>. Namely, if we can assign to a variable of <code>this</code>
     * type a value of the <code>other</code> type.
     *
     * @param other a type.
     * @return true if <code>this</code> type is compatible with the <code>other</code>.
     */
    boolean isCompatibleWith(JSpearType other);

    /**
     * Returns true if <code>this</code> type represents numerical values.
     *
     * @return true if <code>this</code> type represents numerical values.
     */
    boolean isNumerical();

    /**
     * Returns true if <code>this</code> type represents an error.
     *
     * @return true if <code>this</code> type represents an error.
     */
    boolean isError();


    /**
     * Returns true if <code>this</code> type can be merged with the <code>other</code> type.
     *
     * @return true if <code>this</code> type can be merged with the <code>other</code> type.
     */
    boolean canBeMergedWith(JSpearType other);

    /**
     * Returns true if <code>this</code> type represents integer values.
     *
     * @return true if <code>this</code> type represents integer values.
     */
    default boolean isInteger() {
        return false;
    }

    /**
     * Returns true if <code>this</code> type represents boolean values.
     *
     * @return true if <code>this</code> type represents boolean values.
     */
    default boolean isBoolean() {
        return false;
    }

    /**
     * Returns true if <code>this</code> type represents real values.
     *
     * @return true if <code>this</code> type represents real values.
     */
    default boolean isReal() {
        return false;
    }

    /**
     * Returns true if <code>this</code> type represents a user defined type.
     *
     * @return true if <code>this</code> type represents a user defined type.
     */
    default boolean isCustom() {
        return false;
    }

    /**
     * Returns true if this type represents random values.
     *
     * @return true if this type represents random values.
     */
    default boolean isRandom() { return false; }

    /**
     * Returns the deterministic version of this type.
     *
     * @return the deterministic version of this type.
     */
    default JSpearType deterministicType() {
        return this;
    }

    /**
     * Returns the value represented by the given parameter.
     *
     * @param v double representation of a value
     * @return the value represented by the given parameter.
     */
    JSpearValue valueOf(double v);

    DataRange getDefaultDataRange();

}
