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

package it.unicam.quasylab.jspear.speclang.values;

import it.unicam.quasylab.jspear.speclang.types.JSpearType;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public sealed interface JSpearValue permits JSpearBoolean, JSPearInteger, JSpearReal, JSpearCustomValue, JSpearErrorValue  {

    JSpearValue ERROR_VALUE = new JSpearErrorValue();

    static boolean isTrue(JSpearValue value) {
        if (value instanceof JSpearBoolean booleanValue) {
            return booleanValue.value();
        }
        return false;
    }

    static int intValue(JSpearValue value) {
        if (value instanceof JSPearInteger integerValue) {
            return integerValue.value();
        }
        return 0;
    }


//    static JSpearValue of(JSpearType type, String variable, DataState ds) {
//        if (type.isAnArray()) {
//            JSpearValue[] elements = IntStream.range(variable.getFirstCellIndex(), variable.getFirstCellIndex()+variable.getSize())
//                    .mapToObj(i -> JSpearValue.realValue(ds.getValue(i)))
//                    .toArray(JSpearValue[]::new);
//            return new JSpearArray(elements);
//        }
//        if (type.isInteger()) {
//            return new JSPearInteger((int) ds.getValue(variable.getFirstCellIndex()));
//        }
//        if (type.isReal()) {
//            return new JSpearReal(ds.getValue(variable.getFirstCellIndex()));
//        }
//        if (type.isBoolean()) {
//            return JSpearBoolean.getBooleanValue(ds.getValue(variable.getFirstCellIndex())>0);
//        }
//        if (type.isCustom()) {
//            return new JSpearCustomValue((JSpearCustomType) type, (int) ds.getValue(variable.getFirstCellIndex()));
//        }
//        return JSpearValue.ERROR_VALUE;
//    }


    JSpearType getJSpearType();

    static JSpearValue sum(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.sum(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.sum(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue product(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.product(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.product(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue subtraction(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.subtraction(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.subtraction(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue division(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.division(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.division(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }


    static JSpearValue modulo(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.modulo(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.modulo(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue apply(DoubleBinaryOperator op, JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.apply(op, v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.apply(op, v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static  JSpearValue apply(DoubleUnaryOperator op, JSpearValue v) {
        if (v instanceof JSPearInteger intValue) {
            return intValue.apply(op);
        }
        if (v instanceof JSpearReal realValue) {
            return realValue.apply(op);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue negate(JSpearValue v) {
        if (v instanceof JSpearBoolean booleanValue) {
            return booleanValue.negate();
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue isLessThan(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.isLessThan(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.isLessThan(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue isLessOrEqualThan(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.isLessOrEqualThan(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.isLessOrEqualThan(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue isEqualTo(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.isEqualTo(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.isEqualTo(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }


    static JSpearValue isGreaterOrEqualThan(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.isGreaterOrEqualThan(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.isGreaterOrEqualThan(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue isGreaterThan(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSPearInteger intValue) {
            return intValue.isGreaterThan(v2);
        }
        if (v1 instanceof JSpearReal realValue) {
            return realValue.isGreaterThan(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }


    static JSpearValue and(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSpearBoolean booleanValue) {
            return booleanValue.and(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }

    static JSpearValue or(JSpearValue v1, JSpearValue v2) {
        if (v1 instanceof JSpearBoolean booleanValue) {
            return booleanValue.or(v2);
        }
        return JSpearValue.ERROR_VALUE;
    }


     static JSpearValue sampleNormal(RandomGenerator rg, JSpearValue v1, JSpearValue v2) {
        return new JSpearReal(rg.nextDouble()*doubleOf(v1)+doubleOf(v2));
    }

    static JSpearValue sample(RandomGenerator rg, JSpearValue from, JSpearValue to) {
        double fromValue = doubleOf(from);
        double gapValue = doubleOf(to)-fromValue;
        return new JSpearReal(fromValue+rg.nextDouble()*gapValue);
    }

    static double doubleOf(JSpearValue v) {
        if (v instanceof JSPearInteger integerValue) {
            return integerValue.value();
        }
        if (v instanceof JSpearReal realValue) {
            return realValue.value();
        }
        return Double.NaN;
    }

    static JSpearValue ifThenElse(JSpearValue eval, Supplier<JSpearValue> v1, Supplier<JSpearValue> v2) {
        if (eval instanceof JSpearBoolean booleanValue) {
            return (booleanValue.value()?v1.get(): v2.get());
        } else {
            return JSpearValue.ERROR_VALUE;
        }
    }

    double toDouble();
}
