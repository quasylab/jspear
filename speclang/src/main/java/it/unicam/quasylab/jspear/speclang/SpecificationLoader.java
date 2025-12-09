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
package it.unicam.quasylab.jspear.speclang;

import it.unicam.quasylab.jspear.SystemSpecification;
import it.unicam.quasylab.jspear.speclang.parsing.*;
import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.function.Consumer;

public class SpecificationLoader {

    private final ParseErrorCollector errors = new ParseErrorCollector();


    public enum ElementType {
        VARIABLES_DECLARATION,
        ENVIRONMENT_DECLARATION, CONTROLLER_DECLARATION
    }


    private ParseTree getParseTree(CharStream source) {
        JSpearSpecificationLanguageLexer lexer = new JSpearSpecificationLanguageLexer(source);
        CommonTokenStream tokens =  new CommonTokenStream(lexer);
        JSpearSpecificationLanguageParser parser = new JSpearSpecificationLanguageParser(tokens);
        ParseErrorListener errorListener = new ParseErrorListener(errors);
        parser.addErrorListener(errorListener);
        JSpearSpecificationLanguageParser.JSpearSpecificationModelContext parseTree = parser.jSpearSpecificationModel();
        if (errors.withErrors()) {
            return null;
        } else {
            return parseTree;
        }
    }

    public SystemSpecification loadSpecification(CharStream source) {
        ParseTree parseTree = getParseTree(source);
        if (parseTree != null) {
            return load(parseTree);
        } else {
            return null;
        }
    }

    public SystemSpecification loadSpecification(InputStream code) throws IOException {
        return loadSpecification(CharStreams.fromStream(code));
    }

    public SystemSpecification loadSpecification(String code) {
        return loadSpecification(CharStreams.fromString(code));
    }

    public SystemSpecification loadSpecification(File file) throws IOException {
        return loadSpecification(CharStreams.fromReader(new FileReader(file)));
    }

    private SystemSpecification load(ParseTree model) {
        JSpearModelGenerator generator = new JSpearModelGenerator(errors);
        model.accept(generator);
        if (errors.withErrors()) {
            return null;
        } else {
            return generator.getSystemSpecification();
        }
    }


    private void doTask(Consumer<ParseTree> task, ParseTree model) {
        if (!errors.withErrors()) {
            task.accept(model);
        }
    }

    public List<String> getErrorMessage() {
        return this.errors.getSyntaxErrorList().stream().map(Object::toString).toList();
    }




}
