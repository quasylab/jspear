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
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ExecController;
import it.unicam.quasylab.jspear.controller.Controller;
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
import it.unicam.quasylab.jspear.robtl.BooleanSemanticsVisitor;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;

import org.apache.commons.math3.random.RandomGenerator;

import java.util.*;
import java.util.function.BinaryOperator;
import java.util.stream.Collectors;

/**
 *  Scenario where two cars V1 and V2 driving on a single lane. V1 is behind V2. Only V1 is managed by the controller.
 */
public class SingleLaneTwoCars {

    private static final double RESPONSE_TIME = 1;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_ACCEL_OFFSET = 5;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double IDLE_DELTA = 1;

    // INITIAL VALUES
    private static final double INIT_SPEED_V1 = 0;
    private static final double INIT_DISTANCE_V1_V2 = 100;
    private static final double INIT_ACCEL_V1 = 0;

    private static final double INIT_SPEED_V2 = 0;
    private static final double INIT_ACCEL_V2 = 1;
 
    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 4;
    private static final int FREQUENCY = 2;
    private static final int TIMES_TO_APPLY = 20;

    private static final double BRAKE_CHECK_CHANCE = 0.8;

    // ROBUSTNESS FORMULAE PARAMETERS
    private static final double ETA_CRASH = 0.3; // Maximum acceptable risk of collision
    private static final double ETA_SAFETY_GAP_VIOLATION = 0.2; // Maximum acceptable risk of violating safety gap

    // ENVIRONMENT VARIABLE INDEXES
    private static final int speedV1 = 0;
    private static final int safetyGap = 1;
    private static final int accelV1 = 2;
    private static final int distance = 3;
    private static final int speedV2 = 4;
    private static final int accelV2 = 5;
    private static final int intention = 6;
    private static final int perturbationApplied = 7;

    private static final int NUMBER_OF_VARIABLES = 8;

    // POSSIBLE CONTROLLER INTENTIONS
    private static final double FASTER = 1.0;
    private static final double SLOWER = -1.0;
    private static final double IDLE  = 0.0;

    private static final int EVOLUTION_SEQUENCE_SIZE = 100;
    private static final int PERTURBATION_SCALE = 100;


    public SingleLaneTwoCars(){
        DataState state = getInitialState();
        ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        Perturbation drunkDriver = getDrunkDriverPerturbation(TIMES_TO_APPLY, FREQUENCY);
        Perturbation brakeChecker = getBreakCheckPerturbation(TIMES_TO_APPLY, FREQUENCY);


        EvolutionSequence drunkDriverSequence = sequence.apply(drunkDriver, STARTING_STEP, PERTURBATION_SCALE);
        EvolutionSequence brakeCheckerSequence = sequence.apply(brakeChecker, STARTING_STEP, PERTURBATION_SCALE);

        printSummary(sequence, 30, "UNPERTURBED");
        printSummary(drunkDriverSequence, 30, "DRUNK DRIVER");
        printSummary(brakeCheckerSequence, 30, "BRAKE CHECKER");
        //RobustnessFormula PHINoPerturbation = getSafetyGapViolationFormula(new NonePerturbation());
        //RobustnessFormula PHIBrakeCheck = getSafetyGapViolationFormula(brakeChecker);
        //RobustnessFormula PHIDrunkDriver = getSafetyGapViolationFormula(drunkDriver);

        //PHINoPerturbation = getCrashFormula(new NonePerturbation());
        RobustnessFormula PHIBrakeCheck = getCrashFormula(brakeChecker);
        RobustnessFormula PHIDrunkDriver = getCrashFormula(drunkDriver);

        for(int testStep = 0; testStep < 100; testStep++){
            System.out.print("Step " + testStep + ":  ");
            System.out.print("PHIBrakeCheck "  + new BooleanSemanticsVisitor().eval(PHIBrakeCheck).eval(100, testStep, sequence));
            System.out.print(" PHIDrunkDriver "  + new BooleanSemanticsVisitor().eval(PHIDrunkDriver).eval(100, testStep, sequence));
            //System.out.print(" PHINoPerturbation "  + new BooleanSemanticsVisitor().eval(PHINoPerturbation).eval(100, testStep, sequence));
            System.out.println();
        }
    }

    private void printSummary(EvolutionSequence sequence, int stepsToPrint, String title){
        System.out.printf("%s%n %16s, %16s, %16s, %16s, %16s, %16s, %16s, %16s%n", title, "accelV1", "speedV1", "accelV2", "speedV2", "distance", "safetyGap", "perturbApplied", "intention");
        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<SystemState> dss = sequence.get(i);

            ArrayList<String> s = new ArrayList<>();
            for (int j : new int[]{accelV1, speedV1, accelV2, speedV2, distance, safetyGap, perturbationApplied}) {
                OptionalDouble avg = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(j))).average();
                s.add(String.format("%16.2f,", avg.getAsDouble()));
            }

            double[] intentions = dss.evalPenaltyFunction(ds -> ds.get(intention));
            Double intention_mode = Arrays.stream(intentions).boxed()
                    .collect(Collectors.groupingBy(t -> t, Collectors.counting()))
                    .entrySet()
                    .stream()
                    .reduce(BinaryOperator.maxBy(Comparator.comparingLong(Map.Entry::getValue)))
                    .map(Map.Entry::getKey)
                    .orElseThrow(IllegalArgumentException::new); // statistical mode
            s.add(String.format("%16.2f", intention_mode));
            System.out.println(String.join(" ", s));
        }
    }

    private static RobustnessFormula getCrashFormula(Perturbation perturbation) {
        DataStateExpression penaltyFunction = ds -> ds.get(distance) > 0.0 ? 0.0 : 1.0;
        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(penaltyFunction), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_CRASH
        );
    }

    private static RobustnessFormula getSafetyGapViolationFormula(Perturbation perturbation) {
        DataStateExpression penaltyFunction = ds -> ds.get(distance) > ds.get(safetyGap) ? 0.0 : 1.0;
        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(penaltyFunction), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(
                perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_SAFETY_GAP_VIOLATION
        );
    }


    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();
        values.put(intention, IDLE);
        // INITIAL DATA FOR V1
        values.put(speedV1, INIT_SPEED_V1);
        values.put(accelV1, INIT_ACCEL_V1);

        // INITIAL DATA FOR V2
        values.put(speedV2, INIT_SPEED_V2);
        values.put(accelV2, INIT_ACCEL_V2);

        values.put(distance, INIT_DISTANCE_V1_V2);

        double initialSafetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, INIT_SPEED_V1, INIT_SPEED_V2);
        values.put(safetyGap, initialSafetyGap);

        values.put(perturbationApplied, 0.0);
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

    private Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Control",
                Controller.ifThenElse(
                        (rg, ds) -> ds.get(distance) == ds.get(safetyGap),
                        Controller.doAction( // IDLE
                                (_rg, _ds) -> List.of(new DataStateUpdate(intention, IDLE)),
                                registry.reference("Control")
                        ),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(distance) > ds.get(safetyGap),
                                Controller.doAction( // FASTER
                                        (_rg, _ds) -> List.of(new DataStateUpdate(intention, FASTER)),
                                        registry.reference("Control")
                                ),
                                Controller.doAction( // SLOWER
                                        (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER)),
                                        registry.reference("Control")
                                )
                        )
                )
        );

        return new ExecController(registry.reference("Control"));

    }

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double intent = state.get(intention);
        double newAccelV1;
        if(intent == FASTER){ // Controller wants to do action FASTER
            double offset = rg.nextDouble() * MAX_ACCEL_OFFSET;
            newAccelV1 = MAX_ACCELERATION - offset;
        }else if(intent == SLOWER){ // Controller wants to do action SLOWER
            newAccelV1 = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
        } else if(intent == IDLE) { // Controller wants to do action IDLE
            newAccelV1 = rg.nextDouble() * (2*IDLE_DELTA) - IDLE_DELTA; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            newAccelV1 = 0;
        }
        updates.add(new DataStateUpdate(accelV1, newAccelV1));

        // Reset V2 acceleration to the initial value
        updates.add(new DataStateUpdate(accelV2, INIT_ACCEL_V2));

        includePhysicsUpdates(state, updates);

        updates.add(new DataStateUpdate(perturbationApplied, 0.0));
        return updates;
    }

    private static void includePhysicsUpdates(DataState state, List<DataStateUpdate> updates) {
        double aV1 = state.get(accelV1);
        double sV1 = state.get(speedV1);
        double travelV1 = aV1/2 + sV1;
        double newSpeedV1 = Math.min(Math.max(0,sV1 + aV1), MAX_SPEED);
        updates.add(new DataStateUpdate(speedV1, newSpeedV1));

        double aV2 = state.get(accelV2);
        double sV2 = state.get(speedV2);
        double travelV2 = aV2/2 + sV2;
        double newSpeedV2 = Math.min(Math.max(0,sV2 + aV2), MAX_SPEED);
        updates.add(new DataStateUpdate(speedV2, newSpeedV2));

        double newDistance = state.get(distance) - travelV1 + travelV2;
        updates.add(new DataStateUpdate(distance, newDistance));

        double newSafetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, sV1, sV2);
        updates.add(new DataStateUpdate(safetyGap, newSafetyGap));
    }

    private static Perturbation getDrunkDriverPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, SingleLaneTwoCars::applyDrunkDriverPerturbation));
    }

    private static DataState applyDrunkDriverPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(perturbationApplied, 1.0));

        // Drunk driving: A uniformly random acceleration or braking for V2
        double perturbedAccel = rg.nextDouble()*(MAX_ACCELERATION + MAX_BRAKE) - MAX_BRAKE;
        updates.add(new DataStateUpdate(accelV2, perturbedAccel));

        includePhysicsUpdates(state, updates);

        return state.apply(updates);
    }

    private static Perturbation getBreakCheckPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, SingleLaneTwoCars::applyBreakCheckPerturbation));
    }

    private static DataState applyBreakCheckPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(perturbationApplied, 1.0));

        // There is a BREAK_CHECK_CHANCE probability that V2 does a break check
        // Break check: apply maximum braking this time step
        boolean doBreakCheck = rg.nextDouble() < BRAKE_CHECK_CHANCE;
        if(!doBreakCheck){
            return state; // If there is no break check, return the state intact
        }
        double perturbedAccel = - MAX_BRAKE;
        updates.add(new DataStateUpdate(accelV2, perturbedAccel));

        includePhysicsUpdates(state, updates);

        return state.apply(updates);
    }

}

