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

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

import it.unicam.quasylab.jspear.ds.DataState;
import org.json.JSONObject;

public class Connector {
    private final String baseUrl;
    private final ArrayList<AiState> aiStates;

    public Connector(String baseUrl) {
        this.baseUrl = baseUrl;
        aiStates = new ArrayList<>();
    }

    public AiState getInitialState(){
        HttpURLConnection initConnection = getInitConnection();
        return new AiState(doPOST(initConnection, null), this);
    }

    private HttpURLConnection getInitConnection() {
        HttpURLConnection initConnection;
        try {
            initConnection = (HttpURLConnection) new URL(baseUrl + "/reset").openConnection();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return initConnection;
    }

    protected int addAiStateToHistory(AiState aiState){
        int index = aiStates.size();
        aiStates.add(aiState);
        return index;
    }

    public AiState getAiStateFromHistory(DataState state){
        int index = (int) state.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX);
        return aiStates.get(index);
    }


    public AiState doStep(DataState perturbedState){
        AiState aiState = getAiStateFromHistory(perturbedState);

        if (aiState.isDone() || aiState.isTruncated()) {
            return aiState;
        }
        HttpURLConnection stepConnection = getStepConnection();
        aiState.setPerturbedDataState(perturbedState);
        return new AiState(doPOST(stepConnection, aiState.toJson()), this);
    }

    private HttpURLConnection getStepConnection() {
        HttpURLConnection stepConnection;
        try {
            stepConnection = (HttpURLConnection) new URL(baseUrl + "/step").openConnection();
            stepConnection.setDoOutput(true);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return stepConnection;
    }

    private JSONObject doPOST(HttpURLConnection connection, JSONObject body) {
        try {
            connection.setRequestMethod("POST");
            connection.setRequestProperty("Content-Type", "application/json");

            if (body != null) {
                try (OutputStream os = connection.getOutputStream()) {
                    byte[] input = body.toString().getBytes(StandardCharsets.UTF_8);
                    os.write(input, 0, input.length);
                }
            }

            int responseCode = connection.getResponseCode();
            StringBuilder response = getResponse(connection, responseCode);
            if (responseCode == HttpURLConnection.HTTP_OK) {
                return new JSONObject(response.toString());
            } else {
                throw new RuntimeException("Failed to get a successful response: " + responseCode + " " + response);
            }
        } catch (Exception e) {
            throw new RuntimeException(e.toString());
        }
    }

    private static StringBuilder getResponse(HttpURLConnection connection, int responseCode) throws IOException {
        StringBuilder response = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                responseCode == HttpURLConnection.HTTP_OK
                        ? connection.getInputStream()
                        : connection.getErrorStream(),
                StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                response.append(line.trim());
            }
        }
        return response;
    }
}
