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
import it.unicam.quasylab.jspear.speclang.values.JSpearCustomValue;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;

import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * This class describes a user defined types. Each custom type has a unique name and consists of a set of <i>elements</i>.
 * Each element is associated with an integer value.
 */
public final class JSpearCustomType implements JSpearType {

    private final String customTypeName;

    private final String[] typeElements;

    private final Map<String,Integer> elementsCode;

    /**
     * Creates a new custom type with the given name and with the given elements.
     *
     * @param customTypeName name of the custom type.
     * @param typeElements names of the elements in the type.
     */
    public JSpearCustomType(String customTypeName, String[] typeElements) {
        this.customTypeName = customTypeName;
        this.typeElements = typeElements;
        this.elementsCode = IntStream.range(0, typeElements.length).boxed().collect(Collectors.toMap(i -> typeElements[i], i -> i));
    }

    /**
     * Returns the code assigned with the given name.
     *
     * @param elementName the name of an element in the type.
     * @return the code assigned with the given name or -1 if the name is unknown.
     */
    public int getCode(String elementName) {
        return this.elementsCode.getOrDefault(elementName, -1);
    }

    @Override
    public JSpearType merge(JSpearType other) {
        if (other.deterministicType().equals(this)) {
            return other;
        }
        return JSpearCustomType.ERROR_TYPE;
    }

    @Override
    public boolean isCompatibleWith(JSpearType other) {
        return this.equals(other.deterministicType());
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
        return this.equals(other.deterministicType());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        JSpearCustomType that = (JSpearCustomType) o;
        return customTypeName.equals(that.customTypeName);
    }

    @Override
    public int hashCode() {
        return Objects.hash(customTypeName);
    }

    @Override
    public boolean isCustom() {
        return true;
    }

    @Override
    public JSpearValue valueOf(double v) {
        return new JSpearCustomValue(this, (int) v);
    }

    @Override
    public DataRange getDefaultDataRange() {
        return new DataRange(0, this.typeElements.length);
    }

    @Override
    public String toString() {
        return customTypeName;
    }

    public JSpearValue getValueOf(String name) {
        return new JSpearCustomValue(this, getCode(name));
    }

    public JSpearCustomValue[] getValues( ) {
        return IntStream.range(0, this.typeElements.length).mapToObj(i -> new JSpearCustomValue(this, i)).toArray(JSpearCustomValue[]::new);
    }

    public String getNameOf(int elementIndex) {
        return this.typeElements[elementIndex];
    }
}
