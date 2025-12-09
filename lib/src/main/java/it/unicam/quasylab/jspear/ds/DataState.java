/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
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
import java.util.List;
import java.util.function.IntToDoubleFunction;
import java.util.function.Predicate;
import java.util.stream.IntStream;

/**
 * A data state is an object associating variables with values.
 *
 */
public class DataState {

    private final double[] data;
    private final DataRange[] dataRanges;
    private double timeStep = 0.0;
    private double granularity = 1.0;
    private double timeReal = 0.0;
    private double timeDelta = 0.0;


    private int step = 0;

    /**
     * Creates a new data state with the given number of cells.
     * By default, all the values are set to 0.0.
     * All the cells in the data state can assume values in the interval
     * [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param size number of cells in the data state.
     */
    public DataState(int size) {
        this(new double[size]);
    }

    /**
     * Creates a new data state with the given values, and uses default values for time variables. All the cells in the data state can assume values
     * in the interval [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param data values in the data state.
     */
    public DataState(double[] data) {
        this(IntStream.range(0, data.length).mapToObj(i -> new DataRange()).toArray(DataRange[]::new), data);
    }

    /**
     * Creates a new data state with the given values, including time variables. All the cells in the data state can assume values
     * in the interval [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param data values in the data state;
     * @param gran initial granularity value;
     * @param Tstep initial time step value;
     * @param Treal initial real time value;
     * @param Tdelta initial value for time delta.
     */
    public DataState(double[] data, double gran, double Tstep, double Treal, double Tdelta) {
        this(IntStream.range(0, data.length).mapToObj(i -> new DataRange()).toArray(DataRange[]::new), data);
        this.granularity = gran;
        this.timeStep = Tstep;
        this.timeReal = Treal;
        this.timeDelta = Tdelta;
    }

    /**
     * Creates a new data state with the given values, including time variables. All the cells in the data state can assume values
     * in the interval [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param data values in the data state;
     * @param Tstep initial time step value.
     */
    public DataState(double[] data, int Tstep) {
        this(IntStream.range(0, data.length).mapToObj(i -> new DataRange()).toArray(DataRange[]::new), data);
        this.step = Tstep;
    }

    /**
     * Creates a new data state with the given number of cells.
     * Values in the data state are initialised by assigning to the cell in position <code>i</code> the value <code>initFunction.applyAsDouble(i)</code>.
     * All the cells in the data state can assume values in the interval
     * [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param size number of cells in the data state.
     * @param initFunction function used to initialise the values.
     */
    public DataState(int size, IntToDoubleFunction initFunction) {
        this(DataRange.getDefaultRangeArray(size), initFunction);
    }

    /**
     * Creates a new data state with the given number of cells. Values in the data state are initialised by
     * assigning to the cell in position <code>i</code> the value <code>initFunction.applyAsDouble(i)</code>.
     * Time variables are initialised individually.
     * All the cells in the data state can assume values in the interval
     * [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param size number of cells in the data state.
     * @param initFunction function used to initialise the values.
     * @param Tstep initial time step value.
     */
    public DataState(int size, IntToDoubleFunction initFunction, int Tstep) {
        this(DataRange.getDefaultRangeArray(size), initFunction);
        this.step = Tstep;
    }

    /**
     * Creates a new data state with the <code>dataRanges.length</code> cells.
     * Values in the data state are initialised by assigning to the cell in position <code>i</code> the value <code>initFunction.applyAsDouble(i)</code>.
     * The cell in position <code>i</code> can assume values in the interval <code>dataRanges[i]</code>.
     *
     * @param initFunction function used to initialise the values.
     * @param dataRanges data ranges of the values in the cells.
     */
    public DataState(DataRange[] dataRanges, IntToDoubleFunction initFunction) {
        this(dataRanges, IntStream.range(0, dataRanges.length).mapToDouble(initFunction).toArray());
    }

    /**
     * Creates a new data state with the given number of cells. Values in the data state are initialised by
     * assigning to the cell in position <code>i</code> the value <code>initFunction.applyAsDouble(i)</code>.
     * Time variables are initialised individually.
     * All the cells in the data state can assume values in the interval
     * [{@link Double#NEGATIVE_INFINITY}, {@link Double#POSITIVE_INFINITY}].
     *
     * @param size number of cells in the data state.
     * @param initFunction function used to initialise the values.
     * @param gran initial granularity value.
     * @param Tstep initial time step value.
     * @param Treal initial real time value.
     * @param Tdelta initial value for time delta.
     */
    public DataState(int size, IntToDoubleFunction initFunction, double gran, double Tstep, double Treal, double Tdelta) {
        this(DataRange.getDefaultRangeArray(size), initFunction);
        this.granularity = gran;
        this.timeStep = Tstep;
        this.timeReal = Treal;
        this.timeDelta = Tdelta;
    }

    /**
     * Creates a new data state with the <code>dataRanges.length</code> cells. Values in the data state are initialised by
     * assigning to the cell in position <code>i</code> the value <code>initFunction.applyAsDouble(i)</code>.
     * The cell in position <code>i</code> can assume values in the interval <code>dataRanges[i]</code>.
     * Default values are used for time variables.
     * Step is initialised individually.
     *
     * @param initFunction function used to initialise the values.
     * @param dataRanges data ranges of the cells.
     * @param Tstep initial time step value.
     */
    public DataState(DataRange[] dataRanges, IntToDoubleFunction initFunction, int Tstep) {
        this(dataRanges, IntStream.range(0, dataRanges.length).mapToDouble(initFunction).toArray());
        this.step = Tstep;
    }

    /**
     * Creates a new data state with <code>dataRanges.length</code> cells
     * that are initialised with the given values <code>data</code>.
     * For any <code>i</code>, <code>dataRanges[i]</code> is the data range for the values in the
     * cell in position <code>i</code>.
     *
     * @param dataRanges data ranges for the values in the cells in the created data state.
     * @param data data state values.
     * @throws IllegalArgumentException if <code>dataRanges.length != data.length</code>.
     */
    public DataState(DataRange[] dataRanges, double[] data) {
        if (dataRanges.length != data.length) {
            throw new IllegalArgumentException();
        }
        this.data = DataRange.apply(dataRanges, data);
        this.dataRanges = dataRanges;
    }

    /**
     * Creates a new data state with the <code>dataRanges.length</code> cells. Values in the data state are initialised by
     * assigning to the cell in position <code>i</code> the value <code>initFunction.applyAsDouble(i)</code>.
     * The cell in position <code>i</code> can assume values in the interval <code>dataRanges[i]</code>.
     * Time variables are initialised individually.
     *
     * @param initFunction function used to initialise the values.
     * @param dataRanges data ranges of the cells.
     * @param gran initial granularity value.
     * @param Tstep initial time step value.
     * @param Treal initial real time value.
     * @param Tdelta initial value for time delta.
     */
    public DataState(DataRange[] dataRanges, IntToDoubleFunction initFunction, double gran, double Tstep, double Treal, double Tdelta) {
        this(dataRanges, IntStream.range(0, dataRanges.length).mapToDouble(initFunction).toArray());
        this.granularity = gran;
        this.timeStep = Tstep;
        this.timeReal = Treal;
        this.timeDelta = Tdelta;
    }

    /**
     * Creates a new data state with <code>dataRanges.length</code> cells that are initialised with the given
     * values <code>data</code>. For any <code>i</code>, <code>dataRanges[i]</code> is the data range for the
     * cell in position <code>i</code>.
     * Default values are used for time variables.
     *
     * @param dataRanges data ranges for the cells in the created data state.
     * @param data data state values.
     * @param Tstep initial time step value.
     * @throws IllegalArgumentException if <code>dataRanges.length != data.length</code>.
     */
    public DataState(DataRange[] dataRanges, double[] data, int Tstep) {
        if (dataRanges.length != data.length) {
            throw new IllegalArgumentException();
        }
        this.data = DataRange.apply(dataRanges, data);
        this.dataRanges = dataRanges;
        this.step = Tstep;
    }

    /**
     * Creates a new data state with <code>dataRanges.length</code> cells that are initialised with the given
     * values <code>data</code>. For any <code>i</code>, <code>dataRanges[i]</code> is the data range for the
     * cell in position <code>i</code>.
     *
     * @param dataRanges data ranges for the cells in the created data state.
     * @param data data state values.
     * @param gran initial granularity value.
     * @param Tstep initial time step value.
     * @param Treal initial real time value.
     * @param Tdelta initial value for time delta.
     * @throws IllegalArgumentException if <code>dataRanges.length != data.length</code>.
     */
    public DataState(DataRange[] dataRanges, double[] data, double gran, double Tstep, double Treal, double Tdelta) {
        if (dataRanges.length != data.length) {
            throw new IllegalArgumentException();
        }
        this.data = DataRange.apply(dataRanges, data);
        this.dataRanges = dataRanges;
        this.granularity = gran;
        this.timeStep = Tstep;
        this.timeReal = Treal;
        this.timeDelta = Tdelta;
    }

    /**
     * Returns the evaluation of the relation <code>></code> between
     * the value in a given cell and a given value.
     *
     * @param idx index of the cell
     * @param value value to which we want to compare the chosen datum
     * @return <code>true</code> if <code>this.data[idx] > value</code>.
     */
    public static Predicate<DataState> greaterThan(int idx, double value) {
        return ds -> ds.get(idx)>value;
    }

    /**
     * Returns the evaluation of the relation <code>>=</code> between
     * the value in a given cell and a given value.
     *
     * @param idx index of the cell
     * @param value value to which we want to compare the chosen datum
     * @return <code>true</code> if <code>this.data[idx] >= value</code>.
     */
    public static Predicate<DataState> greaterOrEqualThan(int idx, double value) {
        return ds -> ds.get(idx)>=value;
    }

    /**
     * Returns the evaluation of the relation <code><</code> between
     * the value in a given cell and a given value.
     *
     * @param idx index of the cell
     * @param value value to which we want to compare the chosen datum
     * @return <code>true</code> if <code>this.data[idx] < value</code>.
     */
    public static Predicate<DataState> lessThan(int idx, double value) {
        return ds -> ds.get(idx)<value;
    }

    /**
     * Returns the evaluation of the relation <code><=</code> between
     * the value in a given cell and a given value.
     *
     * @param idx index of the cell
     * @param value value to which we want to compare the chosen datum
     * @return <code>true</code> if <code>this.data[idx] <= value</code>.
     */
    public static Predicate<DataState> lessOrEqualThan(int idx, double value) {
        return ds -> ds.get(idx)<=value;
    }

    /**
     * Returns the evaluation of the relation <code>==</code> between
     * the value in a given cell and a given value.
     *
     * @param idx index of the cell
     * @param value value to which we want to compare the chosen datum
     * @return <code>true</code> if <code>this.data[idx] = value</code>.
     */
    public static Predicate<DataState> equalsTo(int idx, double value) {
        return ds -> ds.get(idx)==value;
    }

    /**
     * Returns the size of this data state, namely the number of stored values.
     *
     * @return the number of stored values.
     */
    public int size() {
        return this.data.length;
    }

    /**
     * Returns the value in position i.
     * An {@link IndexOutOfBoundsException} is thrown if <code>i<0</code> or <code>i>=this.size()</code>.
     *
     * @param i value index.
     * @return the value in position i
     * @throws IndexOutOfBoundsException if <code>i<0</code> or <code>i>=this.size()</code>.
     */
    public double get(int i) {
        return this.data[i];
    }

    /**
     * Returns an int-to-double function providing a view of the elements in this data space
     * stored in cells with indexes in the interval <code>[from,to]</code>.
     * The returned function associates to each <code>i</code> in the interval <code>[0,to-from]</code>
     * the value contained in the cell with index <code>from+i</code>.
     *
     * @param from starting index of the provided view.
     * @param to   ending index of the provided view.
     * @return an int-to-double function providing a view of the elements in this data space
     * stored in the cells from index <code>from</code> to index <code>to</code>.
     */
    public IntToDoubleFunction get(int from, int to) {
        return i -> {
            if ((i<0)||(i+from>to)||(i+from>this.data.length)) {
                throw new ArrayIndexOutOfBoundsException();
            }
            return this.data[from+i];
        };
    }

    /**
     * Get the values of time variables.
     */
    public double getTimeStep(){
        return this.timeStep;
    }
    public double getTimeReal(){
        return this.timeReal;
    }
    public double getGranularity(){
        return this.granularity;
    }
    public double getTimeDelta(){
        return this.timeDelta;
    }

    /**
     * Get the value of the current time step.
     * @return parameter <code>step</code>.
     */

    public int getStep(){
        return this.step;
    }

    /**
     * Sets the value in position i to the value <code>getDataRange(i).apply(v)</code>.
     *
     * @param i index of cell to set.
     * @param v value to assign to the cell.
     */
    public void set(int i, double v) {
        this.data[i] = this.dataRanges[i].apply(v);
    }

    /**
     * Set the values of time variables.
     */
    public void setTimeStep(double t){
        this.timeStep = t;
    }
    public void setTimeReal(double t){
        this.timeReal = t;
    }
    public void setGranularity(double t){
        this.granularity = t;
    }
    public void setTimeDelta(double t){
        this.timeDelta = t;
    }

    /**
     * Set the value of the current time step.
     *
     * @param newStep new value of the current time step.
     */

    public void setStep(int newStep){
        this.step = newStep;
    }

    /**
     * Returns the data range associated with the cell in position i.
     *
     * @param i index of the cell.
     * @return the data range associated with the cell in position i.
     */
    public DataRange getDataRange(int i) {
        return dataRanges[i];
    }

    /**
     * Returns a new data state with the same data as this data state plus the updates applied.
     *
     * @param updates list of updates to apply.
     * @return the data state obtained from this data state by applying the given updates.
     */
    public DataState apply(List<DataStateUpdate> updates) {
        DataState newDataState = new DataState(this.dataRanges, data);
        updates.forEach(newDataState::apply);
        newDataState.setGranularity(this.getGranularity());
        newDataState.setTimeStep(this.getTimeStep());
        newDataState.setTimeReal(this.getTimeReal());
        newDataState.setTimeDelta(this.getTimeDelta());
        newDataState.setStep(this.getStep());
        return newDataState;
    }

    /**
     * Applies a given update to this data state.
     *
     * @param dataStateUpdate update to be applied.
     */
    private void apply(DataStateUpdate dataStateUpdate) {
        this.set(dataStateUpdate.getIndex(), dataStateUpdate.getValue());
    }

    /**
     * Returns the string representation of this data state.
     * @return The string representation of the array of values.
     */
    public String toString(){
        return Arrays.toString(this.data);
    }



}
