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

import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import org.apache.commons.math3.random.RandomGenerator;
import it.unicam.quasylab.jspear.penalty.*;

import java.util.LinkedList;
import java.util.List;
import java.util.function.*;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * Instances of this class are used to model a set of system states.
 * The set is implemented as a list.
 */
public class SampleSet<T extends SystemState> {

    private final List<T> states;

    /**
     * Creates an empty sample set.
     */
    public SampleSet() {
        this(new LinkedList<>());
    }

    /**
     * Creates a sample set from a given list of system states.
     *
     * @param states system states in the sample.
     */
    public SampleSet(List<T> states) {
        this.states = states;
    }

    /**
     * Returns a set of samples, of a given size, generated with a given function.
     *
     * @param rg random generator
     * @param generator random function used to generate the samples
     * @param size number of samples
     * @return the sample set of size <code>size</code> in which each sample is obtained by applying function <code>generator</code>.
     * @param <T> model domain
     */
    public static <T extends SystemState> SampleSet<T> generate(RandomGenerator rg, Function<RandomGenerator, T> generator, int size) {
        return new SampleSet<>(IntStream.range(0, size).mapToObj(i -> generator.apply(rg)).toList());
    }

    /**
     * Adds a new system state to this sample set.
     *
     * @param state a system state.
     */
    public void add(T state) {
        states.add(state);
    }

    /**
     * Returns the number of items in this sample set.
     *
     * @return the number of items in this sample set.
     */
    public int size() {
        return states.size();
    }

    /**
     * Given a penalty function, described by means of an expression over data states,
     * returns a (sorted) array containing its evaluation on the data state
     * of each element in the sample set.
     *
     * @param f a penalty function.
     * @return a sorted array containing all the evaluations of <code>f</code> over the
     * data states associated to the system states in the sample set.
     */
    public synchronized double[] evalPenaltyFunction(DataStateExpression f) {
        return states.stream().map(SystemState::getDataState).mapToDouble(f).sorted().toArray();
    }

    /**
     * Returns the Wasserstein lifting of a given ground distance on data states,
     * computed according to the functions <code>f</code> and <code>distance</code>,
     * between this sample set and <code>other</code>.
     *
     * @param f penalty function used to compute the ground distance.
     * @param distance ground distance on reals.
     * @param other sample set to compare.
     * @return the Wasserstein lifting of <code>distance</code>,
     * computed on the values obtained by applying <code>f</code> to the data states in the samples,
     * between this sample set and <code>other</code>.
     */
    public synchronized double distance(DataStateExpression f, DoubleBinaryOperator distance, SampleSet<T> other) {
        if (other.size() % this.size() != 0) {
            throw new IllegalArgumentException("Incompatible size of data sets!");
        }
        double[] thisData = this.evalPenaltyFunction(f);
        double[] otherData = other.evalPenaltyFunction(f);
        return computeDistance(distance, thisData, otherData);
    }

    /**
     * In case the ground distance is not given,
     * the Euclidean distance on reals is considered.
     *
     * @param f penalty function on data states.
     * @param other sample set to compare.
     * @return the Wasserstein lifting of the Euclidean distance,
     * computed on the values obtained by applying <code>f</code> to the data states in the samples,
     * between this sample set and <code>other</code>.
     */
    public synchronized double distance(DataStateExpression f, SampleSet<T> other) {
        return distance(f, (v1, v2) -> Math.abs(v2-v1), other);
    }

    /**
     * Utility method to evaluate the Wasserstein distance between two sampled distributions on reals,
     * based on a given ground distance.
     *
     * @param distance ground distance on reals
     * @param thisData an array of real values
     * @param otherData an array of real values
     * @return the Wasserstein lifting of <code>distance</code>
     * between the sampled distributions <code>thisData</code> and <code>otherData</code>.
     */
    private double computeDistance(DoubleBinaryOperator distance, double[] thisData, double[] otherData) {
        int k = otherData.length / thisData.length;
        return IntStream.range(0, thisData.length).parallel()
                .mapToDouble(i -> IntStream.range(0, k).mapToDouble(j -> distance.applyAsDouble(thisData[i],otherData[i * k + j])).sum())
                .sum() / otherData.length;
    }

    /**
     * In case the ground distance is not specified,
     * the Euclidean distance on reals is used.
     *
     * @param thisData an array of real values
     * @param otherData an array of real values
     * @return the Wasserstein lifting of Euclidean distance on reals
     * between the sampled distributions <code>thisData</code> and <code>otherData</code>.
     */
    private double computeDistance(double[] thisData, double[] otherData) {
        return computeDistance((v1, v2) -> Math.abs(v2-v1), thisData, otherData);
    }

    /**
     * Returns the asymmetric distance between <code>other</code> and this sample set computed according to
     * the function <code>f</code>.
     * The cardinality of <code>other</code> must be a multiple of that of this sample set.
     * @param f penalty function used to compute the distance.
     * @param other sample set to compare.
     * @return the distance between <code>other</code> and this sample set computed according to
     * the function <code>f</code>.
     */
    public synchronized double distanceLeq(DataStateExpression f, SampleSet<T> other) {
        return distance(f, (v1,v2) -> Math.max(0.0, v2-v1), other);
    }

    public synchronized double distanceLeq(Penalty rho, SampleSet<T> other, int step) {
        if (other.size() % this.size() != 0) {
            throw new IllegalArgumentException("Incompatible size of data sets!");
        }
        DataStateExpression f = rho.effectUpTo(step).get(step);
        double[] thisData = this.evalPenaltyFunction(f);
        double[] otherData = other.evalPenaltyFunction(f);
        int k = otherData.length / thisData.length;
        return IntStream.range(0, thisData.length).parallel()
                .mapToDouble(i -> IntStream.range(0, k).mapToDouble(j -> Math.max(0,otherData[i * k + j] - thisData[i])).sum())
                .sum() / otherData.length;
    }

    /**
     * Utility method to evaluate the Wasserstein distance between two sampled distributions on reals,
     * based on an asymmetric ground distance.
     *
     * @param thisData an array of real values
     * @param otherData an array of real values
     * @return the asymmetric Wasserstein distance between the sampled distributions <code>thisData</code> and <code>otherData</code>.
     */
    private double computeDistanceLeq(double[] thisData, double[] otherData) {
        return computeDistance((v1, v2) -> Math.max(0.0,v2-v1), thisData, otherData);
    }

    /**
     * Returns the asymmetric distance between this sample set and <code>other</code> computed according to
     * the function <code>f</code>.
     * The cardinality of <code>other</code> must be a multiple of that of this sample set.
     * @param f penalty function used to compute the distance.
     * @param other sample set to compare.
     * @return the distance between this sample set and <code>other</code> computed according to
     * the function <code>f</code>.
     */
    public synchronized double distanceGeq(DataStateExpression f, SampleSet<T> other) {
        return distance(f, (v1,v2) -> Math.max(0, v1-v2), other);
    }

    public synchronized double distanceGeq(Penalty rho, SampleSet<T> other, int step) {
        if (other.size() % this.size() != 0) {
            throw new IllegalArgumentException("Incompatible size of data sets!");
        }
        DataStateExpression f = rho.effectUpTo(step).get(step);
        double[] thisData = this.evalPenaltyFunction(f);
        double[] otherData = other.evalPenaltyFunction(f);
        int k = otherData.length / thisData.length;
        return IntStream.range(0, thisData.length).parallel()
                .mapToDouble(i -> IntStream.range(0, k).mapToDouble(j -> Math.max(0, thisData[i] - otherData[i * k + j])).sum())
                .sum() / otherData.length;
    }

    /**
     * Utility method to evaluate the Wasserstein distance between two sampled distributions on reals,
     * based on an asymmetric ground distance.
     *
     * @param thisData an array of real values
     * @param otherData an array of real values
     * @return the asymmetric Wasserstein distance between the sampled distributions <code>otherData</code> and <code>thisData</code>.
     */
    private double computeDistanceGeq(double[] thisData, double[] otherData) {
        return computeDistance((v1, v2) -> Math.max(0.0,v1-v2), thisData, otherData);
    }

    /**
     * Returns the confidence interval of the evaluation of the distance between this sample set and <code>other</code>
     * computed according to the function <code>f</code>.
     * The confidence interval is evaluated by means of the empirical bootstrap method.
     *
     * @param rg a random generator
     * @param f penalty function used to compute the distance.
     * @param other sample set to compare.
     * @param m number of applications of bootstrapping
     * @param z the desired quantile of the standard-normal distribution
     * @return the limits of the confidence interval of the evaluation of the distance between this sample set and <code>other</code> computed according to
     * the function <code>f</code>.
     */
    public synchronized double[] bootstrapDistance(RandomGenerator rg, DataStateExpression f, ToDoubleBiFunction<double[], double[]> distanceFunction, SampleSet<T> other, int m, double z) {
        if (other.size()%this.size()!=0) {
            throw new IllegalArgumentException("Incompatible size of data sets!");
        }
        double[] W = new double[m];
        double WSum = 0.0;
        double[] thisData = this.evalPenaltyFunction(f);
        double[] otherData = other.evalPenaltyFunction(f);
        for (int i = 0; i<m; i++){
            double[] thisBootstrapData = IntStream.range(0, thisData.length).mapToDouble(j -> thisData[rg.nextInt(thisData.length)]).sorted().toArray();
            double[] otherBootstrapData = IntStream.range(0, otherData.length).mapToDouble(j -> otherData[rg.nextInt(otherData.length)]).sorted().toArray();
            W[i] = distanceFunction.applyAsDouble(thisBootstrapData, otherBootstrapData);
            WSum += W[i];
        }
        double BootMean = WSum/m;
        double StandardError = Math.sqrt(IntStream.range(0,m).mapToDouble(j->Math.pow(W[j]-BootMean,2)).sum()/(m-1));
        double[] CI = new double[2];
        CI[0] = Math.max(0,BootMean - z*StandardError);
        CI[1] = Math.min(BootMean + z*StandardError,1);
        return CI;
    }

    /**
     * In case the random generator is not passed as parameter,
     * the default one is used.
     */
    public synchronized double[] bootstrapDistance(DataStateExpression f, ToDoubleBiFunction<double[], double[]> distanceFunction, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(new DefaultRandomGenerator(), f, distanceFunction, other, m, z);
    }

    /**
     * In case the method to compute the distance is not passed as parameter,
     * method <code>computeDistance</code> is used as default.
     */
    public synchronized double[] bootstrapDistance(RandomGenerator rg, DataStateExpression f, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(rg, f, this::computeDistance, other, m , z);
    }

    /**
     * In case neither the random generator nor the distance method are passed as parameters,
     * the default ones are used.
     */
    public synchronized double[] bootstrapDistance(DataStateExpression f, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(new DefaultRandomGenerator(), f, this::computeDistance, other, m, z);
    }

    /**
     * Returns the confidence interval of the evaluation of the asymmetric distance between
     * <code>other</code> and this sample set computed according to the function <code>f</code>.
     * The confidence interval is evaluated by means of the empirical bootstrap method.
     *
     * @param f penalty function used to compute the distance.
     * @param other sample set to compare.
     * @param m number of applications of bootstrapping
     * @param z the desired quantile of the standard-normal distribution
     * @return the limits of the confidence interval of the evaluation of the distance between this sample set and <code>other</code> computed according to
     * the function <code>f</code>.
     */
    public synchronized double[] bootstrapDistanceLeq(RandomGenerator rg, DataStateExpression f, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(rg, f, this::computeDistanceLeq, other, m , z);
    }

    /**
     * In case the random generator is not passed as parameter,
     * the default one is used.
     */
    public synchronized double[] bootstrapDistanceLeq(DataStateExpression f, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(new DefaultRandomGenerator(), f, this::computeDistanceLeq, other, m, z);
    }

    /**
     * Returns the confidence interval of the evaluation of the asymmetric distance between
     * this sample set and <code>other</code> computed according to the function <code>f</code>.
     * The confidence interval is evaluated by means of the empirical bootstrap method.
     *
     * @param f penalty function used to compute the distance.
     * @param other sample set to compare.
     * @param m number of applications of bootstrapping
     * @param z the desired quantile of the standard-normal distribution
     * @return the limits of the confidence interval of the evaluation of the distance between this sample set and <code>other</code> computed according to
     * the function <code>f</code>.
     */
    public synchronized double[] bootstrapDistanceGeq(RandomGenerator rg, DataStateExpression f, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(rg, f, this::computeDistanceGeq, other, m , z);
    }

    /**
     * In case the random generator is not passed as parameter,
     * the default one is used.
     */
    public synchronized double[] bootstrapDistanceGeq(DataStateExpression f, SampleSet<T> other, int m, double z) {
        return bootstrapDistance(new DefaultRandomGenerator(), f, this::computeDistanceGeq, other, m, z);
    }

    /**
     * Returns a sequential stream of this sample set.
     *
     * @return a sequential stream of this sample set.
     */
    public Stream<T> stream() {
        return states.stream();
    }

    /**
     * Returns a sample set obtained by applying the given operator to all the elements of this sample set.
     * @param function operator to apply.
     * @return a sample set obtained by applying <code>function</code> to all the elements of this sample set.
     */
    public SampleSet<T> apply(UnaryOperator<T> function) {
        return new SampleSet<>(this.stream().parallel().map(function).toList());
    }

    /**
     * Returns a new sample set obtained by applying a given function to all the elements of this sample set.
     * @param rg random generator used to sample random values.
     * @param function function used to generate a new element.
     * @return a new sample set obtained by applying <code>function</code> to all the elements of this sample set.
     */
    public SampleSet<T> apply(RandomGenerator rg, BiFunction<RandomGenerator, T, T> function) {
        return new SampleSet<>(
                this.stream().parallel().map(s -> function.apply(rg, s)).toList()
        );
    }

    /**
     * Returns a sample set obtained from this one by replicating all the elements the given number of times.
     *
     * @param k number of copies.
     * @return a sample set obtained from this one by replicating all the elements <code>k</code>  times.
     */
    public SampleSet<T> replica(int k) {
        return new SampleSet<>(
                this.stream().flatMap(e -> IntStream.range(0, k).mapToObj(i -> e)).toList()
        );
    }

    public SampleSet<T> applyDistribution(RandomGenerator rg, DataStateFunction function, boolean parallel){
        if(parallel){
            return new SampleSet<>(this.stream().parallel().map(s -> (T) s.apply(rg, function)).toList());
        } else {
            return new SampleSet<>(this.stream().map(s -> (T) s.apply(rg, function)).toList());
        }
    }

    public double mean(ToDoubleFunction<T> function){
        return this.stream().mapToDouble(function).average().orElse(0.0);
    }

    public SampleSet<SystemState> applyDistribution(RandomGenerator rg, DataStateFunction function){
        return new SampleSet<>(this.stream().parallel().map(s -> s.apply(rg, function)).toList());
    }
}