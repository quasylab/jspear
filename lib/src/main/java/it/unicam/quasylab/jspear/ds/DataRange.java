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

import java.util.Arrays;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;

/**
 * We use this record to assign a range to each value stored in a data state.
 *
 * @param minValue minimal value that can be assigned
 * @param maxValue maximal value that can be assigned
 */
public record DataRange(double minValue, double maxValue) {

    /**
     * If the range is not specified, the default range interval
     * <code>[-infinity,+infinity]</code> is used.
     */
    public DataRange() {
        this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /**
     * Generates an array of a given size of default data ranges.
     *
     * @param size size of the array
     * @return the array with <code>size</code> default data ranges.
     */
    public static DataRange[] getDefaultRangeArray(int size) {
        return IntStream.range(0, size).mapToObj(i -> new DataRange()).toArray(DataRange[]::new);
    }

    /**
     * Returns the values to be stored in the cells
     * after their compliance with the ranges has been ensured via <code>dataRanges[i].apply(data[i])</code>.
     *
     * @param dataRanges data ranges associated to each cell
     * @param data values to be stored in the cells
     * @return the array with the values to be stored in each cell once they comply with the range.
     */
    public static double[] apply(DataRange[] dataRanges, double[] data) {
        return IntStream.range(0, dataRanges.length).mapToDouble(i -> dataRanges[i].apply(data[i])).toArray();
    }

    /**
     * Returns the given value if it belongs to this data range.
     * Otherwise, returns the range bound that is closer to the given value.
     *
     * @param v a value
     * @return <code>v</code> if it complies with this data range.
     * Otherwise, returns the lower bound <code>minValue</code> of this range if <code>v < minValue</code>,
     * and the upper bound <code>maxValue</code> if <code>v > maxValue</code>.
     */
    public double apply(double v) {
        return Math.max(minValue, Math.min(maxValue, v));
    }
}
