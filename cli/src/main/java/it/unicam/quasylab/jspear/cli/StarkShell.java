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
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Scanner;
import java.util.function.Predicate;

public class StarkShell implements Runnable {


    private static final String WELCOME_MESSAGE = "Start using Stark...";
    private final PrintStream output;
    private final PrintStream error;

    private final Scanner input;

    private final StarkInterpreter interpreter;

    public StarkShell(PrintStream output, PrintStream error, Scanner input) throws StarkCommandExecutionException {
        this.output = output;
        this.error = error;
        this.input = input;
        this.interpreter = new StarkInterpreter();
    }

    public StarkShell() throws StarkCommandExecutionException {
        this(System.out, System.err, new Scanner(System.in));
    }


    public void run() {
        boolean flag = true;
        showWelcomeMessage();
        while (flag) {
            flag = readAndExecute();
        }
    }

    public void run(List<String> commands) {
        for (String cmd: commands) {
            if (!execute(cmd)) {
                return ;
            }
        }
    }

    private boolean readAndExecute() {
        showPrompt();
        return execute(this.input.nextLine());
    }

    private boolean execute(String cmd) {
        long start = System.currentTimeMillis();
        return showCommandResult(interpreter.executeCommand(cmd), start);
    }

    public void executeScript(String script) throws StarkCommandExecutionException {
        Path path = Path.of(script);
        interpreter.setWorkingDirectory(path.getParent().toFile());
        String[] scriptCommands = getScriptCommands(path);
        for (String cmd: scriptCommands) {
            this.output.println("> "+cmd);
            execute(cmd);
        }
    }

    private String[] getScriptCommands(Path path) throws StarkCommandExecutionException {
        try {
            return Files.lines(path).filter(Predicate.not(String::isBlank)).toArray(String[]::new);
        } catch (IOException e) {
            throw new StarkCommandExecutionException(e.getMessage());
        }
    }


    private boolean showCommandResult(StarkCommandExecutionResult commandResult, long start) {
        long end = System.currentTimeMillis();
        PrintStream stream = (commandResult.result()?this.output:this.error);
        stream.println(commandResult.message());
        for (String str: commandResult.details()) {
            stream.println(str);
        }
        stream.println("Execution Time: "+(end-start)/1000);
        return !commandResult.quit();
    }

    private void showPrompt() {
        this.output.print("> ");
        this.output.flush();
    }

    private void showWelcomeMessage() {
        this.output.println(WELCOME_MESSAGE);
    }

    public static void main(String[] args) {
        if (args.length == 0) {
            startInteractive();
        } else {
            startBatch(args);
        }
    }

    private static void startBatch(String[] scriptFiles) {
        try {
            StarkShell shell = new StarkShell();
            for (String script: scriptFiles) {
                shell.executeScript(script);
            }
        } catch (StarkCommandExecutionException e) {
            System.err.println(e.getMessage());
        }
    }

    private static void startInteractive() {
        try {
            new StarkShell().run();
        } catch (StarkCommandExecutionException e) {
            System.err.println(e.getMessage());
        }
    }
}
