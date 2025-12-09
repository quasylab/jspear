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

package it.unicam.quasylab.jspear;

/// This class can be used to mute the simulation monitor. This monitor will print one line at the start of the simulation and then will not log anything else.
public class SilentMonitor implements SimulationMonitor {


    public SilentMonitor(String label) {
        System.out.println(label+": Monitor is silent");
    }


    @Override
    public void startSamplingsOfStep(int step) {

    }

    @Override
    public void endSamplingsOfStep(int step) {

    }

    @Override
    public boolean hasBeenCancelled() {
        return false;
    }
}
