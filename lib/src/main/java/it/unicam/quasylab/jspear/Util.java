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

package it.unicam.quasylab.jspear;

import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.DataStateExpression;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * This class provides utility methods that implement useful tasks.
 */
public class Util {

    /**
     * Monte Carlo estimation of a distribution over real values.
     * The values are obtained by evaluating a given penalty function
     * over the data states of the given (sampled) system states.
     *
     * @param sampleSet a list of sampled states
     * @param penalty a penalty function
     * @param from left bound of the sampling interval
     * @param to right bound of the sampling interval
     * @param steps number of intervals
     * @return the Monte Carlo estimation of the probability distribution over the values
     * obtained from the evaluation of <code>penalty</code> over the data states
     * of the system states in <code>sampleSet</code>.
     * @param <T> model domain.
     */
    public static <T extends SystemState> double[] estimateProbabilityDistribution(SampleSet<T> sampleSet,
                                                                                   DataStateExpression penalty,
                                                                                   double from,
                                                                                   double to,
                                                                                   int steps) {
        double dt = (to-from)/steps;
        double[] result = sampleSet.evalPenaltyFunction(penalty);
        double size = result.length;
        return IntStream.range(0, steps).mapToDouble(i -> DoubleStream.of(result).filter(x -> x<=from+i*dt).count()/size).toArray();
    }

    /**
     * In case parameters <code>from</code> and <code>to</code> are not specified,
     * the default values, respectively, <code>0</code> and <code>1</code>
     * are used in the estimation.
     *
     * @param sampleSet a set of sampled states
     * @param penalty a penalty function
     * @param steps number of intervals
     * @return the Monte Carlo estimation of the probability distribution over the values
     * obtained from the evaluation of <code>expr</code> over the data states
     * of the system states in <code>sampleSet</code>.
     * @param <T> model domain.
     */
    public static <T extends SystemState> double[] estimateProbabilityDistribution(SampleSet<T> sampleSet,
                                                                                   DataStateExpression penalty,
                                                                                   int steps) {
        return estimateProbabilityDistribution(sampleSet, penalty, 0, 1.0, steps);
    }

    /**
     * Evaluates a given penalty function over the data states in the system states
     * reached in a given time interval in a given evolution sequence.
     *
     * @param sequence an evolution sequence
     * @param from left bound of the time interval
     * @param to right bound of the time interval
     * @param penalty a penalty function
     * @return the array containing, for each time step in <code>[from,to]</code>,
     * the evaluation of <code>penalty</code> over the data states of the states
     * reached in the <code>sequence</code> at that time step.
     * @param <T> model domain.
     */
    public static <T extends SystemState> double[][] evalDataStateExpression(EvolutionSequence sequence, int from, int to, DataStateExpression penalty) {
        return IntStream.range(from, to).mapToObj(i -> sequence.get(i).evalPenaltyFunction(penalty)).toArray(double[][]::new);
    }

    /**
     * In case the left bound <code>from</code> of the time interval is not specified,
     * the default value <code>0</code> is used.
     *
     * @param sequence an evolution sequence
     * @param to right bound of the time interval
     * @param penalty a penalty function
     * @return the array containing, for each time step in <code>[0,to]</code>,
     * the array of the evaluations of <code>penalty</code> over the data states of the states
     * reached in the <code>sequence</code> at that time step.
     * @param <T> model domain.
     */
    public static <T extends SystemState> double[][] evalDataStateExpression(EvolutionSequence sequence, int to, DataStateExpression penalty) {
        return evalDataStateExpression(sequence, 0, to, penalty);
    }

    /**
     * Evaluates the given distance expressions between two evolution sequences
     * at each time step in a given time interval.
     *
     * @param sequence an evolution sequence
     * @param sequence2 an evolution sequence
     * @param from left bound of the time interval
     * @param to right bound of the time interval
     * @param expressions an array of distance expressions
     * @return the array containing, for each time step in <code>[from,to]</code>,
     * the array of the evaluations each distance expression in <code>expressions</code>
     * between <code>sequence</code> and <code>sequence2</code>
     * computed starting from that time step.
     * @param <T> model domain.
     */
    public static <T extends SystemState> double[][] evalDistanceExpression(EvolutionSequence sequence, EvolutionSequence sequence2, int from, int to, DistanceExpression ...  expressions) {
        return IntStream.range(from, to).mapToObj(i -> Stream.of(expressions).mapToDouble(expr -> expr.compute(i, sequence, sequence2)).toArray()).toArray(double[][]::new);
    }

    /**
     * In case the left bound <code>from</code> of the time interval is not specified,
     * the default value <code>0</code> is used.
     *
     * @param sequence an evolution sequence
     * @param sequence2 an evolution sequence
     * @param to right bound of the time interval
     * @param expressions an array of distance expressions
     * @return the array containing, for each time step in <code>[0,to]</code>,
     * the array of the evaluations each distance expression in <code>expressions</code>
     * between <code>sequence</code> and <code>sequence2</code>
     * computed starting from that time step.
     * @param <T> model domain.
     */
    public static <T extends SystemState> double[][] evalDistanceExpression(EvolutionSequence sequence, EvolutionSequence sequence2, int to, DistanceExpression...  expressions) {
        return evalDistanceExpression(sequence, sequence2, 0, to, expressions);
    }

    /**
     * Stores the given data into a csv file.
     *
     * @param fileName name of the csv file
     * @param data the data to be stored in <code>fileName</code>
     * @throws IOException exception.
     */
    public static void writeToCSV(String fileName, double[][] data) throws IOException {
        Files.writeString(Path.of(fileName), stringOfCSV(data));
    }

    /**
     * Returns the translation of a given array of reals into a sequence of characters.
     * @param data an array of reals
     * @return the sequence of characters corresponding to <code>data</code>.
     */
    private static CharSequence stringOfCSV(double[][] data) {
        return Stream.of(data).sequential().map(row -> DoubleStream.of(row).sequential().mapToObj(d -> ""+d).collect(Collectors.joining(", "))).collect(Collectors.joining("\n"));
    }
}
