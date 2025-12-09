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

package it.unicam.quasylab.jspear.controller;

import java.util.HashMap;
import java.util.Map;

/**
 * A utility class that can be used to define different controllers.
 *
 */
public class ControllerRegistry {

    public static final Controller NIL = new NilController();

    private final Map<String, Controller> controllers;

    /**
     * Creates an empty registry.
     */
    public ControllerRegistry() {
        this.controllers = new HashMap<>();
    }

    /**
     * Sets the controller associated with the given name.
     *
     * @param name a name.
     * @param controller controller associated with the given name.
     */
    public void set(String name, Controller controller) {
        this.controllers.put(name, controller);
    }

    /**
     * Returns the controller associated with the given name. If no controller is defined with that name,
     * a {@link NilController} is returned.
     *
     * @param name a name.
     * @return the controller associated with the given name.
     */
    public Controller get(String name) {
        return this.controllers.getOrDefault(name, NIL);
    }

    /**
     * Returns a controller that refers to the one defined in this registry with the given name.
     * @param name a name.
     * @return a controller that refers to the one defined in this registry with the given name.
     */
    public Controller reference(String name) {
        return (rg, ds) -> get(name).next(rg, ds);
    }

}
