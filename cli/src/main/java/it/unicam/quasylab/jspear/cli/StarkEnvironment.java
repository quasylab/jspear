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

/**
 * Instances of this class acts as an interpreter of Stark commands.
 */
package it.unicam.quasylab.jspear.cli;

import it.unicam.quasylab.jspear.SystemSpecification;
import it.unicam.quasylab.jspear.robtl.TruthValues;
import it.unicam.quasylab.jspear.speclang.SpecificationLoader;

import java.io.File;
import java.io.IOException;
import java.util.stream.Stream;

public class StarkEnvironment {


    private static final String LOAD_ERROR_MESSAGE = "An error occurred while loading specification.";
    private static final String NOSPECIFICATION_IS_LOADED = "No STARK specification has been loaded";
    private static final int DEFAULT_SCALE = 10;
    private SystemSpecification specification;

    private int scale = DEFAULT_SCALE;


    public boolean loadSpecification(File fileName) throws StarkCommandExecutionException {
        SpecificationLoader loader = new SpecificationLoader();
        try {
            SystemSpecification loadedSpecification = loader.loadSpecification(fileName);
            if (loadedSpecification == null) {
                throw new StarkCommandExecutionException(LOAD_ERROR_MESSAGE, loader.getErrorMessage());
            } else {
                this.specification = loadedSpecification;
                return true;
            }
        } catch (IOException e) {
            throw new StarkCommandExecutionException(e);
        }
    }

    public String[] getFormulas() throws StarkCommandExecutionException {
        if (specification != null) {
            return specification.getFormulas();
        } else {
            throw new StarkCommandExecutionException(NOSPECIFICATION_IS_LOADED);
        }
    }

    public String[] getPenalties() throws StarkCommandExecutionException {
        if (specification != null) {
            return specification.getPenalties();
        } else {
            throw new StarkCommandExecutionException(NOSPECIFICATION_IS_LOADED);
        }
    }

    public void setSize(int size) throws StarkCommandExecutionException {
        checkSpecification();
        this.specification.setSize(size);
    }

    public double[][] simulate(int size, int deadline, String selected) {
        return null;
    }


    public boolean loadSpecification(String fileName) throws StarkCommandExecutionException {
        return loadSpecification(new File(fileName));
    }

    public String[] getDistances() throws StarkCommandExecutionException {
        if (specification != null) {
            return specification.getDistanceExpressions();
        } else {
            throw new StarkCommandExecutionException(NOSPECIFICATION_IS_LOADED);
        }
    }

    public void clear() {
        this.specification.clear();
    }

    public double[] compute(String distance, String perturbation, int at, int[] steps) throws StarkCommandExecutionException {
        checkSpecification();
        checkPerturbation(perturbation);
        return specification.evalDistanceExpression(distance, perturbation, at, scale, steps);
    }

    private void checkPerturbation(String perturbation) throws StarkCommandExecutionException {
        if (specification.getPerturbation(perturbation) == null) {
            throw new StarkCommandExecutionException(StarkMessages.unknownPerturbation(perturbation));
        }
    }

    private void checkSpecification() throws StarkCommandExecutionException {
        if (specification == null) {
            throw new StarkCommandExecutionException(NOSPECIFICATION_IS_LOADED);
        }
    }

    private void checkPenalty(String name) throws StarkCommandExecutionException {
        if (specification.getPenalty(name) == null) {
            throw new StarkCommandExecutionException(StarkMessages.unknownPenalty(name));
        }
    }

    private void checkFormula(String name) throws StarkCommandExecutionException {
        if (specification.getFormula(name) == null) {
            throw new StarkCommandExecutionException(StarkMessages.unknownFormula(name));
        }
    }

    public double[][] eval(String penalty, int[] steps) throws StarkCommandExecutionException {
        checkSpecification();
        checkPenalty(penalty);
        return specification.evalPenalty(penalty, steps);
    }

    public double[] checkThreeValued(String formula, int[] steps) throws StarkCommandExecutionException {
        checkSpecification();
        checkFormula(formula);
        return Stream.of(specification.evalThreeValuedSemantic(formula, this.scale, steps)).mapToDouble(TruthValues::valueOf).toArray();
    }

    public double[] checkBoolean(String formula, int[] steps) throws StarkCommandExecutionException {
        checkSpecification();
        checkFormula(formula);
        return Stream.of(specification.evalBooleanSemantic(formula, this.scale, steps)).mapToDouble(b -> (b?1.0:-1.0)).toArray();
    }

    public int getSize() {
        if (specification == null) {
            return SystemSpecification.DEFAULT_SIZE;
        } else {
            return specification.getSize();
        }
    }

    public int getM() {
        if (specification == null) {
            return SystemSpecification.DEFAULT_M;
        } else {
            return specification.getM();
        }
    }

    public double getZ() {
        if (specification == null) {
            return SystemSpecification.DEFAULT_Z;
        } else {
            return specification.getZ();
        }
    }

    public int getScale() {
        return this.scale;
    }

    public void setScale(int scale) {
        this.scale = scale;
    }

    public void setM(int m) throws StarkCommandExecutionException {
        checkSpecification();
        this.specification.setM(m);
    }

    public void setZ(double z) throws StarkCommandExecutionException {
        checkSpecification();
        this.specification.setZ(z);
    }

    public void setRandomSeed(long seed) throws StarkCommandExecutionException {
        checkSpecification();
        this.specification.setRand(seed);
    }

    public String[] getPerturbations() throws StarkCommandExecutionException {
        checkSpecification();
        return this.specification.getPerturbations();
    }
}
