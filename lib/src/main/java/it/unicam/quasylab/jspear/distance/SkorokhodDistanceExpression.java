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
    
    private int previousOffset;
    private final boolean direction;
    private final int rightBound;
    private final int leftBound;
    private final int offsetEvaluationCount;
    private final int scanWidth;

    private final int[] usedOffsets;

    /**
     * Generates the atomic distance expression that will use the given penalty function
     * and the given distance over reals for the evaluation of the ground distance on data states.
     * @param rho the penalty function
     * @param distance ground distance on reals.
     * @param rho2 for normalizing time
     * @param leftBound step from which to start evaluating: returns regular wasserstein distance before.
     * @param rightBound number of steps to be simulated
     * @param direction direction to allow time jumps toward, true = forward, false = backward.
     * @param offsetEvaluationCount number of offsets/lambda functions that will be evaluated/considered
     * @param scanWidth number of steps that will be evaluated when determining max distance according to lambda function
     */
    public SkorokhodDistanceExpression(DataStateExpression rho, DoubleBinaryOperator distance, ToDoubleFunction<Integer> rho2, int leftBound, int rightBound, boolean direction, int offsetEvaluationCount, int scanWidth) {
        this.rho = rho;
        this.rho2 = rho2;
        this.distance = distance;
        this.direction = direction;
        this.previousOffset = 0;
        this.offsetEvaluationCount = offsetEvaluationCount;
        this.rightBound = rightBound;
        this.leftBound = leftBound;
        this.scanWidth = scanWidth;

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

        double _distance;

        int _offset = FindLambdaSkorokhod(step, seq1, seq2, this.offsetEvaluationCount);

        int offset1 = 0;
        int offset2 = 0;
        if (direction)  // if forward direction, iterate over seq2, else seq 1
        {
            offset2 = _offset;
        }
        else
        {
            offset1 = _offset;
        }

        _distance = seq1.get(step + offset1).distance(this.rho, this.distance, seq2.get(step + offset2));

        // for analysis
        this.usedOffsets[step] = _offset;

        return _distance;
    }

    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        throw new UnsupportedOperationException("Not implemented yet");

        // double[] res = new double[3];
        // res[0] = seq1.get(step).distance(this.rho, this.distance, seq2.get(step));
        // ToDoubleBiFunction<double[],double[]> bootDist = (a,b)->IntStream.range(0, a.length).parallel()
        //         .mapToDouble(i -> IntStream.range(0, b.length/a.length).mapToDouble(j -> distance.applyAsDouble(a[i],b[i * (b.length/a.length) + j])).sum())
        //         .sum() / b.length;
        // double[] partial = seq1.get(step).bootstrapDistance(rg, this.rho, bootDist, seq2.get(step),m,z);
        // res[1] = partial[0];
        // res[2] = partial[1];
        // return res;
    }

    
    /**
     * Finds offset from which sequence 2 should be sampled, using skorokhod metric
     * 
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @param offsetEvaluationCount number of offsets/lambda functions that will be evaluated/considered
     * @return the offset at which sequence 2 should be sampled when measuring wasserstein distance between both sequences using skorokhod metric.
     */
    private int FindLambdaSkorokhod(int step, EvolutionSequence seq1, EvolutionSequence seq2, int offsetEvaluationCount)
    {
        // TODO: Test negative direction

        // Do not consider an offset before leftBound
        if (step < this.leftBound)
        {
            return 0;
        }

        // if this is one of the last steps in the simulation.
        if (step + previousOffset >= rightBound)
        {
            // TODO: Discuss correct behavior. For now, just compare to closest valid state. Returning UNKNOWN could be better.
            return (rightBound - 1) - step; // return offset so that sampled step is the last one in sequence 2.
        }

        int offset = previousOffset;
        double smallestmu = Double.MAX_VALUE;

        // disallow picking an offset earlier than a previously used offset, so start sampling from the previous offset.
        // then, find shortest normalized distance, taking both wasserstein distance, and time distance into account.
        // stop scanning when all seq2 steps were analyzed, or scanWidth is reached.
        for (int i = previousOffset; i <= previousOffset + offsetEvaluationCount; i++) {

            System.out.print(i); // for debug
            System.out.print(" ");

            // find Max distance over time given this lambda/offset:
            double sampledDistance = EvaluateLambda(step, this.scanWidth ,seq1, seq2, i);

            System.out.print("max: " + sampledDistance + " | ");    // also debug, best to comment this out to avoid painful eyes

            // calculate time offset that was used:
            double timeOffset = rho2.applyAsDouble(i);
            
            // skorokhod logic:
            double mu = Math.max(timeOffset, sampledDistance);
            
            // if a shorter mu is found, save according data.
            if (mu < smallestmu)
            {
                smallestmu = mu;
                offset = i;
            }

            // if next iteration would be out of range of rightbound, scanning is finished, stop for-loop.
            // Or, if the found mu is 0, a shorter one will not be found. Stop for-loop
            if (mu == 0 || step + i >= rightBound)
            {
                i = Integer.MAX_VALUE - 1;
            }
        }

        previousOffset = offset;
        return offset;
    }


    /**
     * Iterates over the sequences, finding the largest distance between the sequences, given a time translation lambda/offset
     * 
     * @param step time step from which the sequences will be evaluated
     * @param range number of steps that will be evaluated
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @param offset the time translation lambda as a constant offset
     * @return the largest found wasserstein distance between the sequences, given the time translation lambda
     */
    private double EvaluateLambda(int step, int range, EvolutionSequence seq1, EvolutionSequence seq2, int offset)
    {
        // TODO: handle negative direction

        int offset1 = 0;
        int offset2 = 0;
        if (direction)  // if forward direction, iterate over seq2, else seq 1
        {
            offset2 = offset;
        }
        else
        {
            offset1 = offset;
        }

        double maxDistance = 0;

        for (int i = 0; i < range; i++) 
        {
            // skip this evaluation if it would sample a negative step
            if (step + i + offset < 0) {
                continue;
            }

            double sampledDistance = seq1.get(step + i + offset1).distance(this.rho, this.distance, seq2.get(step + i + offset2));

            if (sampledDistance > maxDistance)
            {
                maxDistance = sampledDistance;
            }

            // if next iteration would be out of range of rightbound, scanning is finished, stop for-loop.
            if (step + i + offset >= rightBound)
            {
                i = Integer.MAX_VALUE - 1;
            }
        }
        return maxDistance;
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
