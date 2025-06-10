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
public final class draft implements DistanceExpression {

    private final DataStateExpression rho;
    private final ToDoubleFunction<Integer> rho2; // used to normalize time in addition to distance
    private final DoubleBinaryOperator distance;
    private final DoubleBinaryOperator muLogic; // used to determine mu from timestamp, and distance
    
    private int previousOffset;
    private final int rightBound;
    private final int leftBound;
    private final double maxValue;

    private final int[] usedOffsets;
    private double[][] dpdistances; // to re-use previously calculated wasserstein distances.
    private boolean[][] lastdp;

    /**
     * Generates the atomic distance expression that will use the given penalty function
     * and the given distance over reals for the evaluation of the ground distance on data states.
     * @param rho the penalty function
     * @param distance ground distance on reals.
     * @param muLogic logic to assign weight of sampled offset.
     * @param rho2 for normalizing time
     * @param leftBound step from which to start evaluating: returns regular wasserstein distance before.
     * @param rightBound number of steps to be simulated
     * @param lambdaCount number of offsets/lambda functions that will be evaluated/considered
     * @param scanWidth number of steps that will be evaluated when determining max distance according to lambda function
     */
    public draft(DataStateExpression rho, DoubleBinaryOperator distance, DoubleBinaryOperator muLogic ,ToDoubleFunction<Integer> rho2,
                                         int leftBound, int rightBound, double maxValue) {
        this.rho = rho;
        this.rho2 = rho2;
        this.distance = distance;
        this.previousOffset = 0;
        this.rightBound = rightBound;
        this.leftBound = leftBound;
        this.muLogic = muLogic;
        this.usedOffsets = new int[rightBound];
        this.maxValue = maxValue;

        this.dpdistances = new double[rightBound - leftBound][rightBound - leftBound];
        
        initdp(rightBound - leftBound, rightBound - leftBound);
    }

    private void initdp(int m, int n)
    {
        // initialize dynamic programming tables
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                dpdistances[i][j] = Double.MAX_VALUE;
            }
        }
    }

    private double sampleDistance(int leftBound, int i, int j, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        if (dpdistances[i][j] == Double.MAX_VALUE) {
            dpdistances[i][j] = seq1.get(i + leftBound).distance(this.rho, this.distance, seq2.get(j + leftBound));
        }
        return dpdistances[i][j];
    }

    private double dist(int i, int j)
    {
        return Math.abs(rho2.applyAsDouble(i - j));
    }

    private boolean isReachable(int leftBound, int rightBound, EvolutionSequence seq1, EvolutionSequence seq2, double delta) {
        int m = rightBound - leftBound;
        int n = m;
        boolean[][] dp = new boolean[m][n];

        System.out.print("delta: " + delta + "\n");

        // Initialize
        dp[0][0] = this.muLogic.applyAsDouble(sampleDistance(leftBound,0,0,seq1,seq2), dist(0, 0)) <= delta;

        // Fill DP table
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                // distance at i,j must <= delta
                if (this.muLogic.applyAsDouble(sampleDistance(leftBound, i, j, seq1, seq2), dist(i + leftBound, j + leftBound)) > delta) continue;

                // if dist(i,j) <= delta, only assign true if a previous sample <= delta
                if (i > 0 && dp[i - 1][j]) dp[i][j] = true;
                if (j > 0 && dp[i][j - 1]) dp[i][j] = true;
                if (i > 0 && j > 0 && dp[i - 1][j - 1]) dp[i][j] = true;
            }
        }

        this.lastdp = dp;

        return dp[m - 1][n - 1];
    }

    public double computeSkorokhodDistance(double max, int leftBound, int rightBound, EvolutionSequence seq1, EvolutionSequence seq2, double tol) {
        double low = 0.0;
        double high = max + 1.0; // initial upper bound

        while (high - low > tol) {
            double mid = (low + high) / 2.0;
            if (isReachable(leftBound, rightBound, seq1, seq2, mid)) {
                high = mid;
            } else {
                low = mid;
            }
        }
        return high;
    }

    public int[] computeOffsetWarpingPath() {
        boolean[][] dp = this.lastdp;
        int[] path = new int[dp.length];
        int i = dp.length - 1;
        int j = dp[0].length - 1;
        
        // if (!dp[i][j]) {
        //     throw new IllegalStateException("No valid warping path exists (dp[m-1][n-1] == false)");
        // }

        while (i > 0 || j > 0) {
            path[i] = j - i;

            if (i > 0 && j > 0 && dp[i - 1][j - 1]) {
                i--; j--;
            } else if (i > 0 && dp[i - 1][j]) {
                i--;
            } else if (j > 0 && dp[i][j - 1]) {
                j--;
            } else {
                throw new IllegalStateException("Broken DP trace at (" + i + ", " + j + ")");
            }
        }

        return path;
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
        return computeSkorokhodDistance(this.maxValue, this.leftBound, this.rightBound, seq1, seq2, Double.MIN_VALUE);
    }

    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        
        throw new UnsupportedOperationException("Not implemented yet");

        // find best fitting offset
        // double boop =  computeSkorokhodDistance(this.maxValue, this.leftBound, this.rightBound, seq1, seq2, Double.MIN_VALUE);

        // sample bootstrap distance using offset
        // double[] res = bootstrapSample(rg, step, offset, seq1, seq2, m, z);

        // // for analysis
        // this.usedOffsets[step] = offset;
        // System.out.println("step: "+ step + "Distance: " + res[0]);
        // return boop;
        // return new double[0];
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
