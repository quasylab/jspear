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

package it.unicam.quasylab.jspear;

/**
 * This class can be used to monitor simulations.
 */
public class ConsoleMonitor implements SimulationMonitor {

    private final String label;
    private long last;

    /**
     * Assigns a name to the simulation.
     *
     * @param label a string corresponding to the simulation name.
     */
    public ConsoleMonitor(String label) {
        this.label = label;
    }

    @Override
    public void startSamplingsOfStep(int step) {
        System.out.println(label+": Sampling of step "+step+" started.");
        last = System.currentTimeMillis();
    }

    @Override
    public void endSamplingsOfStep(int step) {
        long elapsed = System.currentTimeMillis() - last;
        System.out.println(label+": Sampling of step "+step+" completed.");
        System.out.println(label+": Elapsed time "+(elapsed/1000.0)+"s");
    }

    @Override
    public boolean hasBeenCancelled() {
        return false;
    }
}
