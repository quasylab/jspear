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
import java.io.IOException;
import java.util.List;

public class StarkCommandExecutionException extends Exception {

    public static final String ILLEGAL_COMMAND = "ERROR: Illegal command!";
    private static final String FILE_DOES_NOT_EXIST = "ERROR: %s does not exist!";

    private static final String FILE_IS_NOT_A_DIRECTORY = "ERROR: %s is not a directory!";

    private final List<String> reasons;



    public StarkCommandExecutionException(String message, List<String> reasons) {
        super(message);
        this.reasons = reasons;
    }

    public StarkCommandExecutionException(Exception e) {
        super(e);
        this.reasons = List.of();
    }

    public StarkCommandExecutionException(String message) {
        this(message, List.of());
    }

    public static String fileDoesNotExists(File file) {
        return String.format(FILE_DOES_NOT_EXIST, file.getAbsolutePath());
    }

    public static String fileIsNotADirectory(File file) {
        return String.format(FILE_IS_NOT_A_DIRECTORY, file.getAbsolutePath());

    }


    public List<String> getReasons() {
        return this.reasons;
    }
}
