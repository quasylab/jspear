/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
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

package it.unicam.quasylab.jspear.speclang.parsing;

import java.util.LinkedList;
import java.util.List;

public class ParseErrorCollector {

    private final LinkedList<ParseError> errors;

    public ParseErrorCollector() {
        this.errors = new LinkedList<>();
    }

    public int size() {
        return errors.size();
    }

    public boolean withErrors() {
        return !errors.isEmpty();
    }

    public synchronized void record(ParseError error) {
        errors.add(error);
    }

    public synchronized List<ParseError> getSyntaxErrorList() {
        return new LinkedList<>(errors);
    }
}
