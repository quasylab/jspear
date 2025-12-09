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

import java.io.File;
import java.util.List;

public class StarkMessages {


    private static final String WORKING_DIRECTORY_MESSAGE = "Working directory: %s";
    private static final String LIST_MESSAGE = "Content:";
    private static final String ILLEGAL_ACCESS = "An error occurred while accessing to %s";
    private static final String LOAD_MESSAGE = "Specification %s has been successfully loaded.";
    private static final String QUIT_MESSAGE = "See you next time!";
    private static final String FORMULAS_MESSAGE = "Formulas:";
    private static final String PENALTIES_MESSAGE = "Penalties:";
    private static final String DISTANCES_MESSAGE = "Distance expressions:";
    private static final String DONE_MESSAGE = "Done";
    private static final String PRINT_MESSAGE = "Values:";
    private static final String NO_DATA_TO_PRINT = "Error: No data to print!";
    private static final String NO_DATA_TO_SAVE = "Error: No data to save!";
    private static final String UNKNOWN_PERTURBATION_MESSAGE = "Error: perturbation %s is unknown!";
    private static final String UNKNOWN_DISTANCE_MESSAGE = "Error: distance %s is unknown!";
    private static final String UNKNOWN_FORMULA_MESSAGE = "Error: formula %s is unknown!";
    private static final String UNKNOWN_PENALTY_MESSAGE = "Error: penalty %s is unknown!";;
    private static final String INFO_MESSAGE = "Info: ";
    private static final String SIZE_MESSAGE = "size = %d";
    private static final String SCALE_MESSAGE = "scale = %d";
    private static final String M_MESSAGE = "m = %d";
    private static final String Z_MESSAGE = "z = %f";

    public static final String HELP_MESSAGE = "Stark Commands:";

    public static final List<String> HELP_DETAILS = List.of(
    );
    private static final String PERTURBATION_MESSAGE = "Perturbations: ";

    public static String currentWorkingDirectory(File newDirectory) {
        return String.format(WORKING_DIRECTORY_MESSAGE, newDirectory.getAbsolutePath());
    }

    public static String listMessage() {
        return LIST_MESSAGE;
    }

    public static String illegalAccess(File file) {
        return String.format(ILLEGAL_ACCESS, file.getAbsolutePath());
    }

    public static String loadMessage(String fileName) {
        return String.format(LOAD_MESSAGE, fileName);
    }

    public static String quitMessage() {
        return QUIT_MESSAGE;
    }

    public static String formulasMessage() {
        return FORMULAS_MESSAGE;
    }

    public static String penaltiesMessage() {
        return PENALTIES_MESSAGE;
    }

    public static String distancesMessage() {
        return DISTANCES_MESSAGE;
    }

    public static String doneMessage() {
        return DONE_MESSAGE;
    }

    public static String doneMessage(String message) {
        return message;
    }

    public static String printMessage() {
        return PRINT_MESSAGE;
    }

    public static String noDataToPrintMessage() {
        return NO_DATA_TO_PRINT;
    }

    public static String noDataToSaveMessage() {
        return NO_DATA_TO_SAVE;
    }

    public static String unknownPerturbation(String name) {
        return String.format(UNKNOWN_PERTURBATION_MESSAGE, name);
    }

    public static String unknownDistance(String name) {
        return String.format(UNKNOWN_DISTANCE_MESSAGE, name);
    }
    public static String unknownFormula(String name) {
        return String.format(UNKNOWN_FORMULA_MESSAGE, name);
    }


    public static String unknownPenalty(String name) {
        return String.format(UNKNOWN_PENALTY_MESSAGE, name);
    }

    public static String infoMessage() {
        return INFO_MESSAGE;
    }

    public static String sizeValue(int size) {
        return String.format(SIZE_MESSAGE, size);
    }

    public static String scaleValue(int scale) {
        return String.format(SCALE_MESSAGE, scale);
    }

    public static String mValue(int m) {
        return String.format(M_MESSAGE, m);
    }

    public static String zValue(double z) {
        return String.format(Z_MESSAGE, z);
    }

    public static String perturbationMessage() {
        return PERTURBATION_MESSAGE;
    }
}
