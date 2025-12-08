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


package it.unicam.quasylab.jspear.ds;


import java.util.function.Predicate;

/**
 * This functional interface is used to model an expression over a data state.
 */
@FunctionalInterface
public interface DataStateBooleanExpression extends Predicate<DataState> {

    @Override
    default boolean test(DataState state) {return eval(state);}


    boolean eval(DataState state);




}