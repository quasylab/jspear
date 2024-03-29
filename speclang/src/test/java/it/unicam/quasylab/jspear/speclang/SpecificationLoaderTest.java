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

package it.unicam.quasylab.jspear.speclang;

import it.unicam.quasylab.jspear.SystemSpecification;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

class SpecificationLoaderTest {

    public static final String RANDOM_WALK = "./RandomWalk.jspec";
    public static final String ENGINE = "./Engine.jspec";
    public static final String VEHICLE = "./Vehicle.jspec";

    @Test
    void loadRandomWalkSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(RANDOM_WALK)).openStream());
        assertNotNull(spec);
    }
    @Test
    @Disabled
    void loadEngineSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(ENGINE)).openStream());
        assertNotNull(spec);
    }
    @Test
    @Disabled
    void loadVehicleSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        assertNotNull(spec);
    }
}