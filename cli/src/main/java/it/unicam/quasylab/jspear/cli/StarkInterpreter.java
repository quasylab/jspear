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

import it.unicam.quasylab.jspear.speclang.parsing.ParseErrorCollector;
import it.unicam.quasylab.jspear.speclang.parsing.ParseErrorListener;
import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.Token;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class StarkInterpreter {

    private final StarkEnvironment starkEnvironment;

    private File workingDirectory;
    private double[][] lastResults;

    private int[] steps;


    public StarkInterpreter() throws StarkCommandExecutionException {
        this(System.getProperty("user.dir"));
    }


    public StarkInterpreter(String workingDirectory) throws StarkCommandExecutionException {
        this(new File(workingDirectory));
    }


    public StarkInterpreter(File workingDirectory) throws StarkCommandExecutionException {
        setWorkingDirectory(workingDirectory);
        this.starkEnvironment = new StarkEnvironment();
    }

    public void setWorkingDirectory(File workingDirectory) throws StarkCommandExecutionException {
        if (!workingDirectory.exists()) {
            throw new StarkCommandExecutionException(StarkCommandExecutionException.fileDoesNotExists(workingDirectory));
        }
        if (!workingDirectory.isDirectory()) {
            throw new StarkCommandExecutionException(StarkCommandExecutionException.fileIsNotADirectory(workingDirectory));
        }
        this.workingDirectory = workingDirectory;
    }

    public StarkCommandExecutionResult executeCommand(String cmd) {
        try {
            return executeCommand(parseCommand(CharStreams.fromString(cmd)));
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult executeCommand(StarkScriptParser.ScriptCommandContext cmd) {
        return cmd.accept(new StarkCommandVisitor());
    }


    private StarkScriptParser.ScriptCommandContext parseCommand(CharStream source) throws StarkCommandExecutionException {
        ParseErrorCollector errors = new ParseErrorCollector();
        StarkScriptParser.ScriptCommandContext result = getParser(errors, source).scriptCommand();
        if (errors.withErrors()) {
            throw new StarkCommandExecutionException(StarkCommandExecutionException.ILLEGAL_COMMAND, errors.getSyntaxErrorList().stream().map(Object::toString).toList());
        } else {
            return result;
        }
    }

    private StarkScriptParser.StarkScriptContext parseScript(CharStream source) throws StarkCommandExecutionException {
        ParseErrorCollector errors = new ParseErrorCollector();
        StarkScriptParser.StarkScriptContext result = getParser(errors, source).starkScript();
        if (errors.withErrors()) {
            throw new StarkCommandExecutionException(StarkCommandExecutionException.ILLEGAL_COMMAND, errors.getSyntaxErrorList().stream().map(Object::toString).toList());
        } else {
            return result;
        }
    }

    private StarkScriptParser getParser(ParseErrorCollector errors, CharStream source) {
        StarkScriptLexer lexer = new StarkScriptLexer(source);
        CommonTokenStream tokens =  new CommonTokenStream(lexer);
        StarkScriptParser parser = new StarkScriptParser(tokens);
        ParseErrorListener errorListener = new ParseErrorListener(errors);
        parser.addErrorListener(errorListener);
        return parser;
    }

    public class StarkCommandVisitor extends StarkScriptBaseVisitor<StarkCommandExecutionResult> {

        @Override
        public StarkCommandExecutionResult visitChangeDirectoryCommand(StarkScriptParser.ChangeDirectoryCommandContext ctx) {
            return changeDirectory(getFileName(ctx.target.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitListCommand(StarkScriptParser.ListCommandContext ctx) {
            return list();
        }

        @Override
        public StarkCommandExecutionResult visitCwdCommand(StarkScriptParser.CwdCommandContext ctx) {
            return cwd();
        }

        @Override
        public StarkCommandExecutionResult visitLoadCommand(StarkScriptParser.LoadCommandContext ctx) {
            return load(getFileName(ctx.target.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitQuitCommand(StarkScriptParser.QuitCommandContext ctx) {
            return quit();
        }

        @Override
        public StarkCommandExecutionResult visitFormulasCommand(StarkScriptParser.FormulasCommandContext ctx) {
            return formulas();
        }

        @Override
        public StarkCommandExecutionResult visitPenaltiesCommand(StarkScriptParser.PenaltiesCommandContext ctx) {
            return penalties();
        }

        @Override
        public StarkCommandExecutionResult visitDistancesCommand(StarkScriptParser.DistancesCommandContext ctx) {
            return distances();
        }

        @Override
        public StarkCommandExecutionResult visitPerturbationsCommand(StarkScriptParser.PerturbationsCommandContext ctx) {
            return perturbations();
        }

        @Override
        public StarkCommandExecutionResult visitClearCommand(StarkScriptParser.ClearCommandContext ctx) {
            return clear();
        }

        @Override
        public StarkCommandExecutionResult visitSaveCommand(StarkScriptParser.SaveCommandContext ctx) {
            return save(getFileName(ctx.target.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitPrintCommand(StarkScriptParser.PrintCommandContext ctx) {
            return print();
        }

        @Override
        public StarkCommandExecutionResult visitComputeCommand(StarkScriptParser.ComputeCommandContext ctx) {
            return compute(ctx.distance.getText(), ctx.perturbation.getText(), Integer.parseInt(ctx.when.getText()), computeSteps(ctx.steps));
        }

        @Override
        public StarkCommandExecutionResult visitCheckCommand(StarkScriptParser.CheckCommandContext ctx) {
            if (ctx.semantic.getText().equals("boolean")) {
                return checkBoolean(ctx.formula.getText(), computeSteps(ctx.steps));
            } else {
                return checkThreeValued(ctx.formula.getText(), computeSteps(ctx.steps));
            }
        }

        @Override
        public StarkCommandExecutionResult visitEvalCommand(StarkScriptParser.EvalCommandContext ctx) {
            return eval(ctx.penalty.getText(), computeSteps(ctx.steps));
        }

        @Override
        public StarkCommandExecutionResult visitInfoCommand(StarkScriptParser.InfoCommandContext ctx) {
            return info();
        }

        @Override
        public StarkCommandExecutionResult visitSetSizeCommand(StarkScriptParser.SetSizeCommandContext ctx) {
            return setSize(Integer.parseInt(ctx.value.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitSetMCommand(StarkScriptParser.SetMCommandContext ctx) {
            return setM(Integer.parseInt(ctx.value.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitSetZCommand(StarkScriptParser.SetZCommandContext ctx) {
            return setZ(Double.parseDouble(ctx.value.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitSetScaleCommand(StarkScriptParser.SetScaleCommandContext ctx) {
            return setScale(Integer.parseInt(ctx.value.getText()));
        }

        @Override
        public StarkCommandExecutionResult visitSetSeedCommand(StarkScriptParser.SetSeedCommandContext ctx) {
            return setRandomSeed(Long.parseLong(ctx.value.getText()));
        }
    }

    private StarkCommandExecutionResult setSize(int size) {
        try {
            this.starkEnvironment.setSize(size);
            return new StarkCommandExecutionResult(StarkMessages.doneMessage("Done, size correctly set!"), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult setScale(int scale) {
        this.starkEnvironment.setScale(scale);
        return new StarkCommandExecutionResult(StarkMessages.doneMessage("Done, scale correctly set!"), true);
    }

    private StarkCommandExecutionResult setM(int m) {
        try {
            this.starkEnvironment.setM(m);
            return new StarkCommandExecutionResult(StarkMessages.doneMessage("Done, M correctly set!"), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult setZ(double z) {
        try {
            this.starkEnvironment.setZ(z);
            return new StarkCommandExecutionResult(StarkMessages.doneMessage("Done, z correctly set!"), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult setRandomSeed(long seed) {
        try {
            this.starkEnvironment.setRandomSeed(seed);
            return new StarkCommandExecutionResult(StarkMessages.doneMessage("Done, RandomSeed correctly set!"), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult info() {
        return new StarkCommandExecutionResult(
                StarkMessages.infoMessage(),
                List.of(
                        StarkMessages.sizeValue(starkEnvironment.getSize()),
                        StarkMessages.scaleValue(starkEnvironment.getScale()),
                        StarkMessages.mValue(starkEnvironment.getM()),
                        StarkMessages.zValue(starkEnvironment.getZ())
                ),
                true
        );
    }

    private StarkCommandExecutionResult checkThreeValued(String formula, int[] steps) {
        try {
            setLastResults(starkEnvironment.checkThreeValued(formula, steps));
            this.steps = steps;
            return new StarkCommandExecutionResult(StarkMessages.doneMessage(), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult checkBoolean(String formula, int[] steps) {
        try {
            setLastResults(starkEnvironment.checkBoolean(formula, steps));
            this.steps = steps;
            return new StarkCommandExecutionResult(StarkMessages.doneMessage(), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult eval(String penalty, int[] steps) {
        try {
            setLastResults(starkEnvironment.eval(penalty, steps));
            this.steps = steps;
            return new StarkCommandExecutionResult(StarkMessages.doneMessage(), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private void setLastResults(double[][] data) {
        this.lastResults = data;
    }

    private StarkCommandExecutionResult compute(String distance, String perturbation, int at, int[] steps) {
        try {
            setLastResults(starkEnvironment.compute(distance, perturbation, at, steps));
            this.steps = steps;
            return new StarkCommandExecutionResult(StarkMessages.doneMessage(), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private void setLastResults(double[] data) {
        this.lastResults = DoubleStream.of(data).mapToObj(d -> new double[] {d}).toArray(double[][]::new);
    }

    private int[] computeSteps(StarkScriptParser.StepExpressionContext steps) {
        return steps.accept(new StepExpressionVisitor());
    }

    private StarkCommandExecutionResult save(String fileName) {
        if (this.lastResults != null) {
            try {
                if (!fileName.endsWith(".csv")) {
                    fileName = fileName + ".csv";
                }
                PrintWriter pw = new PrintWriter(new File(this.workingDirectory, fileName));
                for (String line: getResultList()) {
                    pw.println(line);
                }
                pw.flush();
                pw.close();
                return new StarkCommandExecutionResult(StarkMessages.doneMessage(),true);
            } catch (FileNotFoundException e) {
                return new StarkCommandExecutionResult(e.getMessage(),false);
            }
        } else {
            return new StarkCommandExecutionResult(StarkMessages.noDataToSaveMessage(),false);
        }
    }

    private StarkCommandExecutionResult print() {
        if (this.lastResults != null) {
            return new StarkCommandExecutionResult(StarkMessages.printMessage(),getResultList(),true);
        } else {
            return new StarkCommandExecutionResult(StarkMessages.noDataToPrintMessage(),false);
        }
    }

    private List<String> getResultList() {
        String[] dataString = Stream.of(this.lastResults).sequential()
                .map(DoubleStream::of)
                .map(ds -> ds.mapToObj(d -> d+"").collect(Collectors.joining(", "))).toArray(String[]::new);
        return IntStream.range(0, steps.length).mapToObj(i -> steps[i]+", "+dataString[i]).toList();
    }

    private StarkCommandExecutionResult clear() {
        starkEnvironment.clear();
        this.lastResults = null;
        this.steps = null;
        return new StarkCommandExecutionResult(StarkMessages.doneMessage(),true);
    }

    private StarkCommandExecutionResult formulas() {
        try {
            return new StarkCommandExecutionResult(StarkMessages.formulasMessage(), List.of(starkEnvironment.getFormulas()), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult penalties() {
        try {
            return new StarkCommandExecutionResult(StarkMessages.penaltiesMessage(), List.of(starkEnvironment.getPenalties()), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult distances() {
        try {
            return new StarkCommandExecutionResult(StarkMessages.distancesMessage(), List.of(starkEnvironment.getDistances()), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }

    private StarkCommandExecutionResult perturbations() {
        try {
            return new StarkCommandExecutionResult(StarkMessages.perturbationMessage(), List.of(starkEnvironment.getPerturbations()), true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(), e.getReasons(), false);
        }
    }


    private StarkCommandExecutionResult quit() {
        return new StarkCommandExecutionResult(StarkMessages.quitMessage(), true, true);
    }

    private StarkCommandExecutionResult load(String fileName) {
        return load(new File(workingDirectory, fileName));
    }

    private StarkCommandExecutionResult load(File file) {
        try {
            this.starkEnvironment.loadSpecification(file);
            return new StarkCommandExecutionResult(StarkMessages.loadMessage(file.getAbsolutePath()),true);
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(),e.getReasons());
        }
    }

    private String getFileName(String target) {
        return target.substring(1,target.length()-1);
    }

    private StarkCommandExecutionResult cwd() {
        return new StarkCommandExecutionResult(StarkMessages.currentWorkingDirectory(workingDirectory), true);
    }

    private StarkCommandExecutionResult list() {
        String[] content = workingDirectory.list();
        if (content == null) {
            return new StarkCommandExecutionResult(StarkMessages.illegalAccess(workingDirectory), false);
        } else {
            return new StarkCommandExecutionResult(StarkMessages.listMessage(), List.of(content), true);
        }
    }

    private StarkCommandExecutionResult changeDirectory(String dir) {
        try {
            File newDirectory = new File(workingDirectory, dir);
            setWorkingDirectory(newDirectory);
            return cwd();
        } catch (StarkCommandExecutionException e) {
            return new StarkCommandExecutionResult(e.getMessage(),e.getReasons());
        }
    }

    private static class StepExpressionVisitor extends StarkScriptBaseVisitor<int[]> {

        @Override
        public int[] visitStepExpressionTarget(StarkScriptParser.StepExpressionTargetContext ctx) {
            return ctx.steps.stream().map(Token::getText).mapToInt(Integer::parseInt).toArray();
        }

        @Override
        public int[] visitStepExpressionInterval(StarkScriptParser.StepExpressionIntervalContext ctx) {
            int from = Integer.parseInt(ctx.from.getText());
            int to = Integer.parseInt(ctx.to.getText());
            int by = Integer.parseInt(ctx.step.getText());
            return IntStream.range(from, to).sequential().filter(i -> (i-from)%by==0).toArray();
        }
    }
}
