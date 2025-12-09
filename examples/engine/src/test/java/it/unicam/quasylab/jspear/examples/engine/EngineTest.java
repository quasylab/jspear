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

package it.unicam.quasylab.jspear.examples.engine;

import it.unicam.quasylab.jspear.ControlledSystem;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.ds.DataState;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class EngineTest {


    @Test
    public void testCooling() {
        DataState ds = Main.getInitialState(150.0);
        Controller ctrl = Main.getControllerRegistry().get("Ctrl");
        DataState nextDs = ds.apply(ctrl.next(new DefaultRandomGenerator(), ds).effect());
        assertEquals(Main.ON, nextDs.get(Main.cool));
    }

    @Test
    public void testController() {
        DataState ds = Main.getInitialState(150.0);
        Controller ctrl = Main.getController();
        DataState nextDs = ds.apply(ctrl.next(new DefaultRandomGenerator(), ds).effect());
        assertEquals(Main.ON, nextDs.get(Main.cool));
    }

    @Test
    public void testCooling2() {
        DataState ds = Main.getInitialState(99.0);
        Controller ctrl = Main.getControllerRegistry().get("Ctrl");
        DataState nextDs = ds.apply(ctrl.next(new DefaultRandomGenerator(), ds).effect());
        assertEquals(Main.OFF, nextDs.get(Main.cool));
    }

    @Test
    public void testStepControlledSystem() {
        DataState state = Main.getInitialState(99.78);
        Controller ctrl = Main.getController();
        SystemState cs = new ControlledSystem(ctrl, (rg, ds) -> ds.apply(Main.getEnvironmentUpdates(rg, ds)), state);
        cs = cs.sampleNext(new DefaultRandomGenerator());
        assertEquals(Main.OFF, cs.getDataState().get(Main.cool));
        cs = cs.sampleNext(new DefaultRandomGenerator());
        assertEquals(Main.ON, cs.getDataState().get(Main.cool));
    }
}