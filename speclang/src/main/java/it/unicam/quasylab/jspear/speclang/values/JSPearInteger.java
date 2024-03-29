/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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
import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;

public final class JSPearInteger implements JSpearValue {
    private final int value;

    public JSPearInteger(int value) {
        this.value = value;
    }

    @Override
    public JSpearType getJSpearType() {
        return JSpearType.INTEGER_TYPE;
    }

    @Override
    public double[] toDoubleArray() {
        return new double[] { this.value };
    }


    public int value() {
        return value;
    }

    public JSpearValue sum(JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return new JSPearInteger(this.value+ intValue.value);
        }
        if (v instanceof JSpearReal realValue) {
            return new JSpearReal(this.value+ realValue.value());
        }
        if (v instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return elementFunction.apply(d -> this.value+d);
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue product(JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return new JSPearInteger(this.value* intValue.value);
        }
        if (v instanceof JSpearReal realValue) {
            return new JSpearReal(this.value* realValue.value());
        }
        if (v instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return elementFunction.apply(d -> this.value*d);
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue subtraction(JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return new JSPearInteger(this.value- intValue.value);
        }
        if (v instanceof JSpearReal realValue) {
            return new JSpearReal(this.value- realValue.value());
        }
        if (v instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return elementFunction.apply(d -> this.value-d);
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue division(JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return new JSPearInteger(this.value/ intValue.value);
        }
        if (v instanceof JSpearReal realValue) {
            return new JSpearReal(this.value/ realValue.value());
        }
        if (v instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return elementFunction.apply(d -> this.value/d);
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue modulo(JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return new JSPearInteger(this.value%intValue.value);
        }
        if (v instanceof JSpearReal realValue) {
            return new JSpearReal(this.value%realValue.value());
        }
        if (v instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return elementFunction.apply(d -> this.value%d);
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue apply(DoubleBinaryOperator op, JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return new JSpearReal(op.applyAsDouble(this.value, intValue.value));
        }
        if (v instanceof JSpearReal realValue) {
            return new JSpearReal(op.applyAsDouble(this.value, realValue.value()));
        }
        if (v instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return elementFunction.apply(d -> op.applyAsDouble(this.value,d));
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue apply(DoubleUnaryOperator op) {
        return new JSpearReal(op.applyAsDouble(this.value));
    }

    public JSpearValue isLessThan(JSpearValue other) {
        if (other instanceof JSPearInteger intValue) {
            return JSpearBoolean.of(this.value()<intValue.value());
        }
        if (other instanceof JSpearReal realValue) {
            return JSpearBoolean.of(this.value()<realValue.value());
        }
        if (other instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return new JSpearArrayElementPredicate(d -> this.value()<elementFunction.apply(d));
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue isLessOrEqualThan(JSpearValue other) {
        if (other instanceof JSPearInteger intValue) {
            return JSpearBoolean.of(this.value()<=intValue.value());
        }
        if (other instanceof JSpearReal realValue) {
            return JSpearBoolean.of(this.value()<=realValue.value());
        }
        if (other instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return new JSpearArrayElementPredicate(d -> this.value()<=elementFunction.apply(d));
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue isEqualTo(JSpearValue other) {
        if (other instanceof JSPearInteger intValue) {
            return JSpearBoolean.of(this.value()==intValue.value());
        }
        if (other instanceof JSpearReal realValue) {
            return JSpearBoolean.of(this.value()==realValue.value());
        }
        if (other instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return new JSpearArrayElementPredicate(d -> this.value()==elementFunction.apply(d));
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue isGreaterOrEqualThan(JSpearValue other) {
        if (other instanceof JSPearInteger intValue) {
            return JSpearBoolean.of(this.value()>=intValue.value());
        }
        if (other instanceof JSpearReal realValue) {
            return JSpearBoolean.of(this.value()>=realValue.value());
        }
        if (other instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return new JSpearArrayElementPredicate(d -> this.value()>=elementFunction.apply(d));
        }
        return JSpearValue.ERROR_VALUE;
    }

    public JSpearValue isGreaterThan(JSpearValue other) {
        if (other instanceof JSPearInteger intValue) {
            return JSpearBoolean.of(this.value()>intValue.value());
        }
        if (other instanceof JSpearReal realValue) {
            return JSpearBoolean.of(this.value()>realValue.value());
        }
        if (other instanceof JSpearArrayElementSelectionFunction elementFunction) {
            return new JSpearArrayElementPredicate(d -> this.value()>elementFunction.apply(d));
        }
        return JSpearValue.ERROR_VALUE;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        JSPearInteger that = (JSPearInteger) o;
        return value == that.value;
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }
}
