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

package nl.tue.ABZ2025.AI;

import it.unicam.quasylab.jspear.ds.DataState;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.Map;

// features ex: ['presence', 'x', 'y', 'vx', 'vy']


public class AiState {
    // expected features: ['presence', 'x', 'y', 'vx', 'vy']
    private static final int SPEED_COLUMN = 3;
    private static final int POSITION_COLUMN = 1;
    private static final int PRESENCE_COLUMN = 0;

    private int originalValuesCount;

    protected final JSONObject state;

    public AiState(JSONObject state) {
        this.state = state;
    }

    protected JSONObject toJson(){
        return state;
    }

    private JSONArray getCarState(){
        return state.getJSONArray("state");
    }

    public int getCarCount(){
        return this.getCarState().length();
    }

    public int getCrashes(){
        return state.getInt("crashes");
    }

    public JSONArray getFeatures(){
        return state.getJSONArray("features");
    }

    public int[] getRealDataStateSpeedIndexes(){
        return getRealDataStateIndexes(SPEED_COLUMN);
    }

    public int[] getRealDataStatePositionIndexes(){
        return getRealDataStateIndexes(POSITION_COLUMN);
    }

    public int[] getRealDataStatePresenceIndexes(){
        return getRealDataStateIndexes(PRESENCE_COLUMN);
    }

    public int[] getPerturbedDataStateSpeedIndexes(){
        return getPerturbedDataStateIndexes(SPEED_COLUMN);
    }

    public int[] getPerturbedDataStatePositionIndexes(){
        return getPerturbedDataStateIndexes(POSITION_COLUMN);
    }

    public int[] getPerturbedDataStatePresenceIndexes(){
        return getPerturbedDataStateIndexes(PRESENCE_COLUMN);
    }


    public int getControlledVehicleIndex(){
        return 0;
    }

    // The AI reports the simulation is truncated if the established duration is reached
    public boolean isTruncated(){
        return state.getBoolean("truncated");
    }

    // The AI reports the simulation is done if the controlled vehicle crashed
    public boolean isDone(){
        return state.getBoolean("done");
    }

    private int[] getRealDataStateIndexes(int column){
        return getDataStateIndexes(column, 0);
    }

    private int[] getPerturbedDataStateIndexes(int column){
        return getDataStateIndexes(column, originalValuesCount);
    }

    private int[] getDataStateIndexes(int column, int offset) {
        int[] m = new int[this.getCarCount()];
        for (int i = 0; i < this.getCarCount(); i++) {
            m[i] = offset+i*getFeatures().length() + column;
        }
        return m;
    }

    public void setDataState(DataState pds) {
        state.put("state", perturbedDataStateToJson(pds));
    }

    private JSONArray perturbedDataStateToJson(DataState pds){
        int carCount = this.getCarCount();
        int colCount = this.getFeatures().length();
        JSONArray perturbedDataState = new JSONArray(carCount);

        for (int i = 0; i < carCount; i++) {
            JSONArray row = new JSONArray(colCount);
            for (int j = 0; j < colCount; j++) {
                double value = pds.get(originalValuesCount+i*carCount+j);
                row.put(j, value);
            }
            perturbedDataState.put(i, row);
        }
        return perturbedDataState;
    }

    /**
     *
     * @return the observable state
     */
    public DataState getDataState(){
        JSONArray carState = this.getCarState();
        int carCount = carState.length();
        int colCount = this.getFeatures().length();

        Map<Integer, Double> values = new HashMap<>();
        for (int i = 0; i < carCount; i++) {
            JSONArray rows = carState.getJSONArray(i);
            if (rows.length() != colCount){
                System.out.println("AIServer: Rows of different sizes in the received observation");
            }
            for (int j = 0; j < colCount; j++) {
                double value;
                if (j < rows.length()){
                    value = rows.getDouble(j);
                } else {
                    value = Double.NaN;
                }
                values.put(i * colCount + j, value);
            }
        }
        // create a duplicate of all variables to be perturbed
        originalValuesCount = values.size();
        for (int i = 0; i < originalValuesCount; i++){
            values.put(originalValuesCount + i, values.get(i));
        }
        return new DataState(values.size(),  i -> values.getOrDefault(i, Double.NaN));
    }
}
