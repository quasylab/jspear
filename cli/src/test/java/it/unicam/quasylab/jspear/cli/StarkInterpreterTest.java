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

package it.unicam.quasylab.jspear.cli;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

@Disabled
class StarkInterpreterTest {

    public static final String two_vehicles = "two_vehicles.stark";
    public static final String vehicle = "single_vehicle.stark";
    public static final String engine = "Engine.stark";

    @Test
    void shouldBeCreatedWithoutErrors() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        assertNotNull(si);
    }

    @Test
    void shouldGetCwd() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        StarkCommandExecutionResult result = si.executeCommand("cwd");
        assertTrue(result.result());
    }

    @Test
    void shouldGetList() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        StarkCommandExecutionResult result = si.executeCommand("ls");
        assertTrue(result.result());
    }

    @Test
    void shouldChangeDirectory() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        StarkCommandExecutionResult result = si.executeCommand("cd \"build\"");
        assertTrue(result.result());
    }

    @Test
    void shouldChangeDirectoryAndGetList() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        si.executeCommand("cd \"build\"");
        StarkCommandExecutionResult result = si.executeCommand("ls");
        assertTrue(result.result());
        assertEquals(StarkMessages.listMessage(), result.message());
        assertTrue(result.details().size()>0);
    }

    @Test
    void shouldFailWhileLoadingAFileThatDoesNotExist() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        StarkCommandExecutionResult result = si.executeCommand("load \"QQDDRR\"");
        assertFalse(result.result());
    }

    @Test
    void shouldQuit() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        StarkCommandExecutionResult result = si.executeCommand("quit");
        assertTrue(result.result());
        assertTrue(result.quit());
        assertEquals(StarkMessages.quitMessage(), result.message());
    }

    @Test
    void testSingleVehicleRun() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        si.executeCommand("cd \"build\"");
        StarkShell shell = new StarkShell();
        shell.executeScript(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(vehicle)).getFile().replaceFirst("^/(.:/)", "$1"));
    }

    @Test
    void testEngineRun() throws StarkCommandExecutionException {
        StarkInterpreter si = new StarkInterpreter();
        si.executeCommand("cd \"build\"");
        StarkShell shell = new StarkShell();
        shell.executeScript(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(engine)).getFile().replaceFirst("^/(.:/)", "$1"));
    }

}
