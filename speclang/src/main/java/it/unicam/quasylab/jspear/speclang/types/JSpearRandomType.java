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

import java.util.Objects;

/**
 * A class representing a type for random values.
 */
public final class JSpearRandomType implements JSpearType {

    private final JSpearType contentType;

    /**
     * Creates a new type with the given content.
     *
     * @param contentType the type of randomly generated values.
     */
    public JSpearRandomType(JSpearType contentType) {
        if (contentType.isRandom()) {
            this.contentType = contentType.deterministicType();
        } else {
            this.contentType = contentType;
        }
    }

    @Override
    public JSpearType merge(JSpearType other) {
        JSpearType mergedContent;
        if (other instanceof JSpearRandomType) {
            mergedContent = this.contentType.merge(((JSpearRandomType) other).getContentType());
        } else {
            mergedContent = this.contentType.merge(other);
        }
        if (mergedContent.isError()||(mergedContent instanceof JSpearRandomType)) {
            return mergedContent;
        }
        return new JSpearRandomType(mergedContent);
    }

    @Override
    public boolean isCompatibleWith(JSpearType other) {
        if (other instanceof JSpearRandomType) {
            return this.contentType.isCompatibleWith(((JSpearRandomType) other).contentType);
        } else {
            return this.contentType.isCompatibleWith(other);
        }
    }

    @Override
    public boolean isNumerical() {
        return this.contentType.isNumerical();
    }


    @Override
    public boolean isError() {
        return false;
    }

    @Override
    public boolean canBeMergedWith(JSpearType other) {
        if (other instanceof JSpearRandomType) {
            return this.contentType.canBeMergedWith(((JSpearRandomType) other).contentType);
        } else {
            return this.contentType.canBeMergedWith(other);
        }
    }

    /**
     * Returns the type of the randomly generated values.
     *
     * @return  the type of the randomly generated values.
     */
    public JSpearType getContentType() {
        return contentType;
    }

    @Override
    public boolean isRandom() {
        return true;
    }

    @Override
    public boolean isInteger() {
        return this.contentType.isInteger();
    }

    @Override
    public boolean isBoolean() {
        return this.contentType.isBoolean();
    }

    @Override
    public boolean isReal() {
        return this.contentType.isReal();
    }

    @Override
    public boolean isCustom() {
        return this.contentType.isCustom();
    }

    @Override
    public String toString() {
        return JSpearType.RANDOM_TYPE_STRING+"["+contentType.toString()+"]";
    }

    @Override
    public JSpearType deterministicType() {
        return this.contentType;
    }

    @Override
    public JSpearValue valueOf(double v) {
        return this.contentType.valueOf(v);
    }

    @Override
    public DataRange getDefaultDataRange() {
        return this.contentType.getDefaultDataRange();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) return false;
        if (obj == this) return true;
        if (!(obj instanceof JSpearRandomType)) return false;
        return this.contentType.equals(((JSpearRandomType) obj).contentType);
    }

    @Override
    public int hashCode() {
        return Objects.hash(RANDOM_TYPE_STRING, this.contentType);
    }
}
