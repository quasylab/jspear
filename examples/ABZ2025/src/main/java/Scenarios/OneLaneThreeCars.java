/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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

package Scenarios;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ExecController;
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpression;
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpressionLeq;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.distance.MaxIntervalDistanceExpression;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.AtomicRobustnessFormula;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.*;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.stream.Collectors;

public class OneLaneThreeCars {

    private static final double RESPONSE_TIME = 1;

    private  static final int NUMBER_OF_VEHICLES = 3;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_ACCEL_OFFSET = 1;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double IDLE_DELTA = 1;

    // INITIAL VEHICLE VALUES
    private static final double[] INIT_SPEED = {0,0,0};
    private static final double[] INIT_ACCEL = {1, 1, 1};
    private static final double[] INIT_DISTANCE_BETWEEN = {300, 300};
    private static final int CONTROLLED_VEHICLE = 1;

    // ENVIRONMENT VARIABLE INDEXES
    private static final int intention = 0;
    private static final int[] speed = new int[]{1,2,3};
    private static final int[] safety_gap = new int[]{4,5};
    private static final int[] accel = new int[]{6,7,8};
    private static final int[] distance = new int[]{9,10};

    private static final int NUMBER_OF_VARIABLES = 1 + speed.length + safety_gap.length + accel.length + distance.length;

    // POSSIBLE INTENTIONS
    private static final double FASTER = 1.0;
    private static final double SLOWER = -1.0;
    private static final double IDLE  = 0.0;

    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 0;
    private static final int FREQUENCY = 2;
    private static final int TIMES_TO_APPLY = 100;

    private static final double DRUNK_DRIVER_CHANCE = 0.2;
    private static final double BRAKE_CHECK_CHANCE = 0.2;

    private static final int TIME_HORIZON = 200;
    // ROBUSTNESS FORMULAE PARAMETERS
    private static final double ETA_CRASH = 0.01; // Maximum acceptable risk of collision
    private static final double ETA_SAFETY_GAP_VIOLATION = 0.5; // Maximum acceptable risk of violating safety gap

    private static final int EVOLUTION_SEQUENCE_SIZE = 100;
    private static final int PERTURBATION_SCALE = 100;


    public OneLaneThreeCars() {
        DataState state = getInitialState();
        ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        printSummary(sequence, TIME_HORIZON, "UNPERTURBED");
        Perturbation drunkDriver = getDrunkDriverPerturbation(TIMES_TO_APPLY, FREQUENCY);
        Perturbation brakeChecker = getBreakCheckPerturbation(TIMES_TO_APPLY, FREQUENCY);

        EvolutionSequence drunkDriverSequence = sequence.apply(drunkDriver, STARTING_STEP, PERTURBATION_SCALE);
        EvolutionSequence brakeCheckerSequence = sequence.apply(brakeChecker, STARTING_STEP, PERTURBATION_SCALE);


        printSummary(drunkDriverSequence, TIME_HORIZON, "DRUNK DRIVER");
        printSummary(brakeCheckerSequence, TIME_HORIZON, "BRAKE CHECKER");

        AtomicDistanceExpressionLeq crashPenalty = new AtomicDistanceExpressionLeq(getCrashPenaltyFn());
        AtomicDistanceExpressionLeq sgViolationPenalty = new AtomicDistanceExpressionLeq(getSafetyGapEntityPenaltyFn());
        System.out.print("BrakeCheck DrunkDriver  \n");
        for (int testStep = 0; testStep < TIME_HORIZON; testStep++) {

            //System.out.print(testStep);
            System.out.printf("%7.7f,", sgViolationPenalty.compute(testStep, sequence, brakeCheckerSequence));
            System.out.printf("%7.7f", sgViolationPenalty.compute(testStep, sequence, drunkDriverSequence));
            System.out.println();
        }
    }

    private static RobustnessFormula getCrashFormula(Perturbation perturbation) {
        DataStateExpression penaltyFunction = getCrashPenaltyFn();


        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpression(penaltyFunction, (v1,v2)-> Math.abs(v1-v2)), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_CRASH
        );
    }

    private static DataStateExpression getCrashPenaltyFn() {
        // Penalizes when the controlled car crashes with the vehicle in front or behind
        return ds ->
                ds.get(distance[CONTROLLED_VEHICLE]) > 0.0
                        || ds.get(distance[CONTROLLED_VEHICLE - 1]) > 0.0
                        ? 0.0 : 1.0;
    }

    private static RobustnessFormula getSafetyGapViolationFormula(Perturbation perturbation) {
        // Penalizes when the controlled car violates de safety gap with the vehicle in front or behind
        DataStateExpression penaltyFunction = getSafetyGapViolationPenaltyFn();
        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(penaltyFunction), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(
                perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_SAFETY_GAP_VIOLATION
        );
    }

    private static DataStateExpression getSafetyGapViolationPenaltyFn() {
        return ds ->
                ds.get(distance[CONTROLLED_VEHICLE]) > ds.get(safety_gap[CONTROLLED_VEHICLE])
                        && ds.get(distance[CONTROLLED_VEHICLE - 1]) > ds.get(safety_gap[CONTROLLED_VEHICLE - 1])
                        ? 0.0 : 1.0;
    }

    private static DataStateExpression getSafetyGapEntityPenaltyFn() {
        return ds ->
                ds.get(distance[CONTROLLED_VEHICLE]) > ds.get(safety_gap[CONTROLLED_VEHICLE])
                        && ds.get(distance[CONTROLLED_VEHICLE - 1]) > ds.get(safety_gap[CONTROLLED_VEHICLE - 1])
                        ? 0.0
                        : Math.min(1, Math.max(0, Math.max(
                        (ds.get(safety_gap[CONTROLLED_VEHICLE]) - ds.get(distance[CONTROLLED_VEHICLE])) / ds.get(safety_gap[CONTROLLED_VEHICLE]),
                        (ds.get(safety_gap[CONTROLLED_VEHICLE - 1]) - ds.get(distance[CONTROLLED_VEHICLE - 1])) / ds.get(safety_gap[CONTROLLED_VEHICLE - 1]))));
    }

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        // Update controlled vehicle's acceleration based on the controller's intention
        double intent = state.get(intention);
        double newAccel;
        if(intent == FASTER){ // Controller wants to do action FASTER
            double offset = rg.nextDouble() * MAX_ACCEL_OFFSET;
            newAccel = MAX_ACCELERATION - offset;
        }else if(intent == SLOWER){ // Controller wants to do action SLOWER
            newAccel = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
        } else if(intent == IDLE) { // Controller wants to do action IDLE
            newAccel = rg.nextDouble() * (2*IDLE_DELTA) - IDLE_DELTA; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            newAccel = 0;
        }
        updates.add(new DataStateUpdate(accel[CONTROLLED_VEHICLE], newAccel));

        // Make the other cars try to keep the safety gap
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            if (i != CONTROLLED_VEHICLE) {
                double newAccelOtherVehicles;

                // Assume the uncontrolled cars are always doing action FASTER
                double offset = rg.nextDouble() * MAX_ACCEL_OFFSET;
                newAccel = MAX_ACCELERATION - offset;
                newAccelOtherVehicles = newAccel;

////              The following is an implementation of the paper controller proposal for the uncontrolled cars
//                if (i == NUMBER_OF_VEHICLES - 1) {
//                    newAccelOtherVehicles = INIT_ACCEL[i]; // The car in front always accelerates
//                } else {
//                    boolean frontDistanceViolated = state.get(distance[i]) < state.get(safety_gap[i]);
//                    boolean backDistanceViolated = i != 0 && (state.get(distance[i - 1]) < state.get(safety_gap[i - 1]));
//                    boolean backDistanceIsSmaller = i != 0 && state.get(distance[i]) < state.get(distance[i - 1]);
//                    if (frontDistanceViolated) {
//                        if (backDistanceViolated && backDistanceIsSmaller) {
//                            newAccelOtherVehicles = INIT_ACCEL[i];
//                        } else {
//                            newAccelOtherVehicles = -MIN_BRAKE;
//                        }
//                    } else if (state.get(distance[i]) == state.get(safety_gap[i])) {
//                        newAccelOtherVehicles = 0;
//                    } else {
//                        newAccelOtherVehicles = INIT_ACCEL[i];
//                    }
//                }
                updates.add(new DataStateUpdate(accel[i], newAccelOtherVehicles));
            }
        }

        includePhysicsUpdates(state, updates);
        return updates;
    }

    private static Perturbation getDrunkDriverPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, OneLaneThreeCars::applyDrunkDriverPerturbation));
    }


    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();
        values.put(intention, IDLE);

        for (int i = 0; i < NUMBER_OF_VEHICLES; i++){
            values.put(speed[i], INIT_SPEED[i]);
            values.put(accel[i], INIT_ACCEL[i]);
            if (i < NUMBER_OF_VEHICLES - 1){
                values.put(distance[i], INIT_DISTANCE_BETWEEN[i]);
                double initialSafetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, INIT_SPEED[i], INIT_SPEED[i+1]);
                values.put(safety_gap[i], initialSafetyGap);
            }
        }
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

    private static double calculateRSSSafetyDistance(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.0637
         */
        double d1 = responseTime*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*responseTime*responseTime;
        double d3 = Math.pow((rearVehicleSpeed+responseTime*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        // The RSS model assumes vehicles as points, but ABZ case study vehicles have dimensions.
        // We add the distances from each vehicle's center to its front/rear bumpers.
        return rssSafetyDistance + VEHICLE_LENGTH;
    }

    private static Perturbation getBreakCheckPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, OneLaneThreeCars::applyBreakCheckPerturbation));
    }

    private static DataState applyBreakCheckPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();

        // uncontrolled vehicle 2 has a BREAK_CHECK_CHANCE probability of doing a break check
        // Break check: apply maximum braking the next time step
        if (rg.nextDouble() < BRAKE_CHECK_CHANCE) {
                double perturbedAccel = - MAX_BRAKE;
            updates.add(new DataStateUpdate(accel[2], perturbedAccel));
            }
        includePhysicsUpdates(state, updates);

        return state.apply(updates);
    }

    /**
     * Adds the following updates to updates via side effects: Speed, Safety gap, and distance
     * between vehicles
     * @param state current data on which the new values will be based
     * @param updates The list to mutate with the addition of updates
     */
    private static void includePhysicsUpdates(DataState state, List<DataStateUpdate> updates){
        // Update the physical properties of cars
        // We store the last car's speed because safety gaps are updated with the speeds of two vehicles
        double currentAccelBack = state.get(accel[0]);
        double currentSpeedBack = state.get(speed[0]);
        double newSpeedBack = Math.min(Math.max(0, currentSpeedBack + currentAccelBack), MAX_SPEED);
        updates.add(new DataStateUpdate(speed[0], newSpeedBack));
        // We loop over the gaps between cars
        for (int i = 0; i < NUMBER_OF_VEHICLES - 1; i++) {
            // Update speeds
            double currentAccelFront = state.get(accel[i+1]);
            double currentSpeedFront = state.get(speed[i+1]);
            double newSpeedFront = Math.min(Math.max(0, currentSpeedFront + currentAccelFront), MAX_SPEED);
            updates.add(new DataStateUpdate(speed[i+1], newSpeedFront));

            // Update distance between cars. The i-th distance is the distance between the i-th (back) and (i+1)-th (front) vehicles
            double travelBack = currentAccelBack / 2 + currentSpeedBack;
            double travelFront = currentAccelFront / 2 + currentSpeedFront;

            double newDistance = state.get(distance[i]) + travelFront - travelBack;
            updates.add(new DataStateUpdate(distance[i], newDistance));

            // Update safety gap. The i-th safety gap is the safety gap between the i-th and (i+1)-th vehicles.
            // The front vehicle does not have a safety back in front.
            double newSafetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, newSpeedBack, newSpeedFront);
            updates.add(new DataStateUpdate(safety_gap[i], newSafetyGap));

            currentAccelBack = currentAccelFront;
            currentAccelBack = currentAccelFront;
            newSpeedBack = newSpeedFront;
        }
    }

    private void printSummary(EvolutionSequence sequence, int stepsToPrint, String title){
        System.out.printf("%s%n%7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s%n", title, "step", "a0","a1","a2", "s0","s1","s2"," d0"," d1"," sg0", "sg1","intent");

        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<SystemState> dss = sequence.get(i);

            ArrayList<String> s = new ArrayList<>();
            s.add(String.format("%7d,", i));
            int[] variablesToPrint = new int[]{accel[0], accel[1], accel[2], speed[0], speed[1], speed[2], distance[0], distance[1], safety_gap[0], safety_gap[1]};
            for (int j : variablesToPrint) {
                OptionalDouble avg = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(j))).average();
                s.add(String.format("%7.2f,", avg.getAsDouble()));
            }

            double[] intention_s = dss.evalPenaltyFunction(ds -> ds.get(intention));
            Double intention_mode = Arrays.stream(intention_s).boxed()
                    .collect(Collectors.groupingBy(t -> t, Collectors.counting()))
                    .entrySet()
                    .stream()
                    .reduce(BinaryOperator.maxBy(Comparator.comparingLong(Map.Entry::getValue)))
                    .map(Map.Entry::getKey)
                    .orElseThrow(IllegalArgumentException::new); // statistical mode
            s.add(String.format("%7.2f", intention_mode));
            System.out.println(String.join(" ", s));
        }
    }

    private static DataState applyDrunkDriverPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();

        // Drunk driving: A uniformly random acceleration or braking for randomly-selected uncontrolled vehicles
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            if(i != CONTROLLED_VEHICLE && rg.nextDouble() < DRUNK_DRIVER_CHANCE) {
                double perturbedAccel = rg.nextDouble()*(MAX_ACCELERATION + MAX_BRAKE) - MAX_BRAKE;
                updates.add(new DataStateUpdate(accel[i], perturbedAccel));
            }
        }

        includePhysicsUpdates(state, updates);

        return state.apply(updates);
    }

    private void printDataForPlots(EvolutionSequence sequence, int stepsToPrint, String title) {
        System.out.printf("%s%n%7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s, %7s%n", title, "step", "a0", "a1", "a2", "s0", "s1", "s2", " d0", " d1", " sg0", "sg1", "intent");

        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<SystemState> dss = sequence.get(i);

            ArrayList<String> s = new ArrayList<>();
            s.add(String.format("%7d,",i));
            int[] variablesToPrint = new int[]{accel[0],  accel[1], accel[2], speed[0], speed[1], speed[2], distance[0], distance[1], safety_gap[0],safety_gap[1]};
            for (int j : variablesToPrint) {
                OptionalDouble avg = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(j))).average();
                s.add(String.format("%7.2f,", avg.getAsDouble()));
            }

            double[] intention_s = dss.evalPenaltyFunction(ds -> ds.get(intention));
            Double intention_mode = Arrays.stream(intention_s).boxed()
                    .collect(Collectors.groupingBy(t -> t, Collectors.counting()))
                    .entrySet()
                    .stream()
                    .reduce(BinaryOperator.maxBy(Comparator.comparingLong(Map.Entry::getValue)))
                    .map(Map.Entry::getKey)
                    .orElseThrow(IllegalArgumentException::new); // statistical mode
            s.add(String.format("%7.2f", intention_mode));
            System.out.println(String.join(" ", s));
        }
    }

    private Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();
        Controller goFaster = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, FASTER)),
                registry.reference("Control"));
        Controller goSlower = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER)),
                registry.reference("Control")
        );

        Function<DataState, Boolean> isFrontDistanceViolated = (ds) -> ds.get(distance[CONTROLLED_VEHICLE]) < ds.get(safety_gap[CONTROLLED_VEHICLE]);
        Function<DataState, Boolean> isBackDistanceViolated = (ds) -> ds.get(distance[CONTROLLED_VEHICLE - 1]) < ds.get(safety_gap[CONTROLLED_VEHICLE - 1]);

        registry.set("Control",
                Controller.ifThenElse(
                        (rg, ds) ->
                                ds.get(distance[CONTROLLED_VEHICLE - 1]) == ds.get(safety_gap[CONTROLLED_VEHICLE - 1])
                                        && ds.get(distance[CONTROLLED_VEHICLE]) == ds.get(safety_gap[CONTROLLED_VEHICLE]),
                        Controller.doAction(
                                (_rg, _ds) -> List.of(new DataStateUpdate(intention, IDLE)),
                                registry.reference("Control")
                        ),
                        Controller.ifThenElse(
                                (rg, ds) -> isFrontDistanceViolated.apply(ds) && isBackDistanceViolated.apply(ds),
                                Controller.ifThenElse((rg, ds) -> ds.get(distance[CONTROLLED_VEHICLE]) > ds.get(distance[CONTROLLED_VEHICLE-1]), goSlower, goFaster),
                                Controller.ifThenElse((rg, ds) -> isFrontDistanceViolated.apply(ds), goSlower, goFaster)
                        )
                )
        );

        return new ExecController(registry.reference("Control"));
    }

}

