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

package it.unicam.quasylab.jspear.distance;

import java.util.function.DoubleBinaryOperator;
import java.util.function.ToDoubleFunction;
import java.util.stream.IntStream;

import org.apache.commons.math3.random.RandomGenerator;

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.ds.DataStateExpression;

/**
 * Class AtomicDistanceExpression implements the atomic distance expression
 * evaluating the Wasserstein lifting of the ground distance, obtained from
 * the given penalty function over data states and the given distance over reals,
 * between the distributions reached at a given time step
 * by two given evolution sequences.
 */
public final class SkorokhodDistanceExpression implements DistanceExpression {

    private final DataStateExpression rho;
    private final ToDoubleFunction<Integer> rho2; // used to normalize time in addition to distance
    private final DoubleBinaryOperator distance;
    private final DoubleBinaryOperator muLogic; // used to determine mu from timestamp, and distance
    
    private int previousOffset;
    private final boolean direction;
    private final int rightBound;
    private final int leftBound;
    private final int lambdaCount;
    private final int scanWidth;

    private final int[] usedOffsets;

    /**
     * Generates the atomic distance expression that will use the given penalty function
     * and the given distance over reals for the evaluation of the ground distance on data states.
     * @param rho the penalty function
     * @param distance ground distance on reals.
     * @param muLogic logic to assign weight of sampled offset.
     * @param rho2 for normalizing time
     * @param leftBound step from which to start evaluating: returns regular wasserstein distance before.
     * @param rightBound number of steps to be simulated
     * @param direction direction to allow time jumps toward, true = forward, false = backward.
     * @param lambdaCount number of offsets/lambda functions that will be evaluated/considered
     * @param scanWidth number of steps that will be evaluated when determining max distance according to lambda function
     */
    public SkorokhodDistanceExpression(DataStateExpression rho, DoubleBinaryOperator distance, DoubleBinaryOperator muLogic ,ToDoubleFunction<Integer> rho2,
                                         int leftBound, int rightBound, boolean direction, int lambdaCount, int scanWidth) {
        this.rho = rho;
        this.rho2 = rho2;
        this.distance = distance;
        this.direction = direction;
        this.previousOffset = 0;
        this.lambdaCount = lambdaCount;
        this.rightBound = rightBound;
        this.leftBound = leftBound;
        this.scanWidth = scanWidth;
        this.muLogic = muLogic;
        this.usedOffsets = new int[rightBound];
    }

    /**
     * Evaluates the Wasserstein lifting of this ground distance
     * between the distributions reached at a given time step
     * by two given evolution sequences.
     *
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the Wasserstein lifting of the ground distance over data states obtained
     * from <code>this.distance</code> and <code>this.rho</code> between
     * the distribution reached by <code>seq1</code> and that reached by <code>seq2</code>
     * at time <code>step</code>.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {

        // find best fitting offset
        int offset = FindLambdaSkorokhod(step, seq1, seq2, this.lambdaCount);

        // sample wasserstein distance using offset
        double _distance = sample(step, offset, seq1, seq2);

        // for analysis
        this.usedOffsets[step] = offset;
        // System.out.println("step: "+ step + "Distance: " + _distance);

        return _distance;
    }

    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        
        // find best fitting offset
        int offset = FindLambdaSkorokhod(step, seq1, seq2, this.lambdaCount);

        // sample bootstrap distance using offset
        double[] res = bootstrapSample(rg, step, offset, seq1, seq2, m, z);

        // for analysis
        this.usedOffsets[step] = offset;
        System.out.println("step: "+ step + "Distance: " + res[0]);
        return res;
    }

    
    /**
     * Finds offset from which sequence 2 should be sampled, using skorokhod metric
     * 
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @param lambdaCount number of offsets/lambda functions that will be evaluated/considered
     * @return the offset at which sequence 2 should be sampled when measuring wasserstein distance between both sequences using skorokhod metric.
     */
    private int FindLambdaSkorokhod(int step, EvolutionSequence seq1, EvolutionSequence seq2, int lambdaCount)
    {
        // Do not consider an offset before leftBound
        if (step < this.leftBound) {
            return 0;
        }

        // if this is one of the last steps in the simulation.
        if (step + previousOffset >= rightBound) {
            return (rightBound - 1) - step; // return offset so that sampled step is the last one in sequence 2.
        }

        int offset = previousOffset;
        double smallestmu = Double.MAX_VALUE;
        double smallestDistance = Double.MAX_VALUE;
        /* 
         * disallow picking an offset earlier than a previously used offset, so start sampling from the previous offset.
         * then, find shortest normalized distance, taking both wasserstein distance, and time distance into account.
         * stop scanning when all seq2 steps were analyzed, or scanWidth is reached.
         * the first lambda to be evaluated has the largest offset. This is to give 'priority' to smaller offsets, since
         * at each evaluation the smallestmu will be reduced, resulting in more thourough future evaluations, since they
         * keep increasing the offset to stay below smallestmu
         */
        for (int i = previousOffset + lambdaCount; i >= previousOffset; i--) {
            // don't evaluate a lambda that exceeds the bounds.
            if (step + i >= rightBound)
            {
                continue;
            }

            // find Max distance over time given this lambda/offset:
            double sampledDistance = EvaluateLambda(step, this.scanWidth ,seq1, seq2, i, smallestDistance);

            // calculate time offset that was used:
            double timeOffset = rho2.applyAsDouble(i);
            
            // skorokhod logic:
            double mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);
            
            // if a shorter mu is found, save according data.
            if (mu < smallestmu) {
                smallestmu = mu;
                offset = i;
            }

            // if a shorter distance is found, save it.
            if (sampledDistance < smallestDistance) {
                smallestDistance = sampledDistance;
            }

            // if the found mu is 0, a shorter one will not be found. Stop for-loop
            if (mu == 0) {
                break;
            }
        }

        // System.out.println("\n picked: " + offset);

        previousOffset = offset;
        return offset;
    }

    /**
     * Iterates over the sequences, finding the largest distance between the sequences, given a time translation lambda/offset.
     * If a distance larger than currentMaximum is sampled, the offset is allowed to increase up to offsetEvaluationCount, in the
     * hopes of finding a smaller distance. This avoids throwing away valid candidates for lambda since the offset is allowed to increase
     * in future evaluations, but may not decrease.
     * 
     * @param step time step from which the sequences will be evaluated
     * @param range number of steps that will be evaluated
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @param offset the time translation lambda as a constant offset
     * @param currentMaximum The current maximum, used to speed up computation
     * @return the largest found wasserstein distance between the sequences, given the time translation lambda
     */
    private double EvaluateLambda(int step, int range, EvolutionSequence seq1, EvolutionSequence seq2, int offset, double currentMaximum)
    {
        double maxDistance = 0;
        int i = 0;

        while (i < range)
        {
            int currentStep = step + i + offset;

            // skip this evaluation if it would sample a negative step
            if (currentStep < 0) {
                i++;
                continue;
            }

            // Stop if sampling would exceed right bound, scanning is finished
            if (currentStep >= rightBound) {
                break;
            }

            double sampledDistance = sample(step + i, offset, seq1, seq2);

            // if found maximum distance is larger than the current maximum distance by a previous lambda, stop iterating
            // since this lambda is not better
            // however, if increasing the offset leads to a smaller ditance, continue with the increased offset
            if(sampledDistance >= currentMaximum) {
                boolean isFirstStep = (i == 0);
                boolean offsetWithinLimit = offset <= lambdaCount;

                if (!isFirstStep && offsetWithinLimit)
                {
                    offset++;
                    // reevaluate this step with new offset
                    // do not save sampled distance, we hope to find a smaller one
                    continue; // don't increase i
                }
                else
                {
                    // stop iteration, this lambda is worse than another
                    maxDistance = sampledDistance;
                    break;
                }
            }

            // if this sampled distance is the new maximum, save it.
            if (sampledDistance > maxDistance) {
                maxDistance = sampledDistance;
            }

            i++;
        }
        // System.out.print(" | "+ offset);
        // System.out.printf(": %.4f", maxDistance);

        return maxDistance;
    }


    /**
     * Iterates over the sequences, finding the largest distance between the sequences, given a constant time translation lambda/offset
     * 
     * @param step time step from which the sequences will be evaluated
     * @param range number of steps that will be evaluated
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @param offset the time translation lambda as a constant offset
     * @return the largest found wasserstein distance between the sequences, given the time translation lambda
     */
    private double EvaluateLambdaSimple(int step, int range, EvolutionSequence seq1, EvolutionSequence seq2, int offset)
    {
        // ensure sampling remains within bounds
        int bound = step + range;
        if (bound > this.rightBound) {
            bound = this.rightBound - offset; // ensure the offset doesnt sample out of bounds
        }

        // find maximum over range by sampling wasserstein distance, using offset
        return IntStream.range(step, bound).parallel()
                .mapToDouble(i -> sample(step, offset, seq1, seq2))
                .max().orElse(Double.NaN);
    }

    /**
     * Samples wasserstein distance given an offset and 2 sequences
     * 
     * @param step time step at which the sequences will be evaluated
     * @param offset one of the sequences will be sampled at an offset from the other
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @return the wasserstein distance between 2 sequences
     */
    private double sample(int step, int offset, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        // if forward direction, iterate over seq2 by adding the offset to its index
        // else iterate over seq 1
        int indexSeq1 = this.direction ? step           : step + offset;
        int indexSeq2 = this.direction ? step + offset  : step;

        return seq1.get(indexSeq1).distance(this.rho, this.distance, seq2.get(indexSeq2));
    }


    /**
     * Samples bootstrap wasserstein distance given an offset and 2 sequences
     * 
     * @param step time step at which the sequences will be evaluated
     * @param offset one of the sequences will be sampled at an offset from the other
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @return the wasserstein distance between 2 sequences
     */
    private double[] bootstrapSample(RandomGenerator rg, int step, int offset, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z)
    {
        throw new UnsupportedOperationException("Not implemented yet");

        // if forward direction, iterate over seq2 by adding the offset to its index
        // else iterate over seq 1
        // int indexSeq1 = this.direction ? step           : step + offset;
        // int indexSeq2 = this.direction ? step + offset  : step;

        // double[] res = new double[3];

        // res[0] = sample(step, offset, seq1, seq2);

        // ToDoubleBiFunction<double[],double[]> bootDist = (a,b)->IntStream.range(0, a.length).parallel()
        //         .mapToDouble(i -> IntStream.range(0, b.length/a.length).mapToDouble(j -> distance.applyAsDouble(a[i],b[i * (b.length/a.length) + j])).sum())
        //         .sum() / b.length;

        // double[] partial = seq1.get(step).bootstrapDistance(rg, this.rho, bootDist, seq2.get(step),m,z);

        // res[1] = partial[0];
        // res[2] = partial[1];
        // return res;
    }

    /**
     * Returns array containing the used offset per step that was evaluated
     * 
     * @return array containing the used offset per step that was evaluated
     */
    public int[] GetOffsetArray()
    {
        return this.usedOffsets;
    }
}
