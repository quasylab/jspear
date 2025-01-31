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
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;

import org.apache.commons.math3.random.RandomGenerator;

import java.util.*;

/**
 *  Scenario where two cars V1 and V2 driving on a single lane. V1 is behind V2. Only V1 is managed by the controller.
 */
public class TwoLanesTwoCars {

    private static final double RESPONSE_TIME = 2;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double FAST_OFFSET = 2;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double SLOW_OFFSET = 2;
    private static final double IDLE_OFFSET = 0.5;


    // INITIAL VALUES
    private static final double MY_INIT_X = 0;
    private static final double MY_INIT_Y = 2;
    private static final double OTHER_INIT_X = 150;
    private static final double OTHER_INIT_Y = 2;
    private static final double MY_INIT_SPEED = 15;
    private static final double OTHER_INIT_SPEED = 15;


    // VARIABLE INDEXES
    private static final int my_x = 0;
    private static final int my_y = 1;
    private static final int my_speed = 2;
    private static final int intention = 3;
    private static final int my_acc = 4;
    private static final int my_lane = 5;
    private static final int my_move = 6;
    private static final int my_timer = 7;
    private static final int my_position = 8;

    private static final int other_x = 9;
    private static final int other_y = 10;
    private static final int other_speed = 11;
    private static final int other_acc = 12;
    private static final int other_lane = 13;
    private static final int other_move = 14;
    private static final int other_timer = 15;

    private static final int dist = 16;
    private static final int safety_gap = 17;

    private static final int NUMBER_OF_VARIABLES = 18;

    // POSSIBLE CONTROLLER ACTIONS
    private static final double FASTER = 1;
    private static final double SLOWER = -1;
    private static final double IDLE  = 0;
    private static final double LANE_RIGHT = -1;
    private static final double LANE_LEFT = 1;

    public TwoLanesTwoCars(){

        int EVOLUTION_SEQUENCE_SIZE = 1;

        RandomGenerator rand = new DefaultRandomGenerator();
        DataState state = getInitialState();
        ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
        EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, EVOLUTION_SEQUENCE_SIZE);

        ArrayList<String> L = new ArrayList<>();
        L.add("my_x");
        L.add("my_y");
        L.add("intention");
        L.add("my_acc");
        L.add("my_speed");
        L.add("other_acc");
        L.add("other_speed");
        L.add("my_position");
        L.add("my_timer");
        L.add("dist");
        L.add("safety_gap");

        ArrayList<DataStateExpression> F = new ArrayList<>();
        F.add(ds->ds.get(my_x));
        F.add(ds->ds.get(my_y));
        F.add(ds->ds.get(intention));
        F.add(ds->ds.get(my_acc));
        F.add(ds->ds.get(my_speed));
        F.add(ds->ds.get(other_acc));
        F.add(ds->ds.get(other_speed));
        F.add(ds->ds.get(my_position));
        F.add(ds->ds.get(my_timer));
        F.add(ds->ds.get(dist));
        F.add(ds->ds.get(safety_gap));

        int H = 50;

        printLData(rand,L,F,system,H,EVOLUTION_SEQUENCE_SIZE);


    }


    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();
        values.put(my_x,MY_INIT_X);
        values.put(my_y,MY_INIT_Y);
        values.put(my_speed,MY_INIT_SPEED);
        values.put(intention, IDLE);
        values.put(my_acc,IDLE);

        values.put(other_x,OTHER_INIT_X);
        values.put(other_y,OTHER_INIT_Y);
        values.put(other_speed, OTHER_INIT_SPEED);
        values.put(other_acc, FASTER);

        values.put(my_move, 0.0);
        values.put(other_move, 0.0);

        values.put(my_lane,(MY_INIT_Y <=3)?0.0:1.0);
        values.put(other_lane,(OTHER_INIT_Y <=3)?0.0:1.0);

        values.put(my_position,(MY_INIT_X <= OTHER_INIT_X)?-1.0:1.0);

        values.put(my_timer,0.0);
        values.put(other_timer,RESPONSE_TIME-1);

        values.put(dist, Math.sqrt(Math.pow((OTHER_INIT_X-MY_INIT_X),2) + Math.pow((OTHER_INIT_Y-MY_INIT_Y),2)));

        double initialSafetyGap = (MY_INIT_X <= OTHER_INIT_X)?calculateRSSSafetyDistance(RESPONSE_TIME, MY_INIT_SPEED, OTHER_INIT_SPEED):calculateRSSSafetyDistance(RESPONSE_TIME, OTHER_INIT_SPEED, MY_INIT_SPEED);
        values.put(safety_gap, initialSafetyGap);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

    private static double calculateRSSSafetyDistance(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.06374
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
                        DataState.equalsTo(my_lane,1),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(dist) > ds.get(safety_gap),
                                Controller.doAction( // LANE_RIGHT
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move, LANE_RIGHT), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                        registry.reference("Moving_right")
                                ),
                                Controller.ifThenElse(
                                        DataState.equalsTo(my_position,1),
                                        Controller.ifThenElse(
                                                DataState.equalsTo(other_lane,1),
                                                Controller.doAction( // LANE_RIGHT
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move,LANE_RIGHT), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                        registry.reference("Moving_right")
                                                ),
                                                Controller.doAction( // FASTER
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer,RESPONSE_TIME)),
                                                                registry.reference("Idling")
                                                )
                                        ),
                                        Controller.ifThenElse(
                                                DataState.equalsTo(other_lane,1),
                                                Controller.ifThenElse(
                                                        (rg, ds) -> ds.get(dist) == ds.get(safety_gap),
                                                        Controller.doAction( // IDLE
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                                registry.reference("Idling")
                                                        ),
                                                        Controller.doAction( // SLOWER
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                                registry.reference("Idling")
                                                        )
                                                ),
                                                Controller.doAction( // FASTER
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                        registry.reference("Idling")
                                                )
                                        )
                                )
                        ),
                        Controller.ifThenElse(
                                (rg,ds) -> ds.get(dist) > ds.get(safety_gap) || ds.get(my_position) == 1,
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                        registry.reference("Idling")
                                ),
                                Controller.ifThenElse(
                                        DataState.equalsTo(other_lane,0),
                                        Controller.ifThenElse(
                                                (rg,ds) -> ds.get(dist) > ds.get(safety_gap)*0.9,
                                                Controller.doAction( // LANE_LEFT
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move,LANE_LEFT), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                        registry.reference("Moving_left")
                                                ),
                                                Controller.doAction( // SLOWER
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                        registry.reference("Idling")
                                                )
                                        ),
                                        Controller.doAction( // IDLE
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                                registry.reference("Idling")
                                        )
                                )
                        )
                )
        );

        registry.set("Idling",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Idling")),
                        registry.reference("Control")
                )
        );

        registry.set("Moving_right",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer,0),
                        Controller.doTick(registry.reference("Moving_right")),
                        Controller.ifThenElse(
                                (rg,ds) -> ds.get(my_position) == 1 || ds.get(dist) > ds.get(safety_gap),
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                        registry.reference("Idling")
                                ),
                                Controller.ifThenElse(
                                        (rg,ds) -> ds.get(dist) == ds.get(safety_gap),
                                        Controller.doAction( // IDLE
                                        (rg,ds)-> List.of(new DataStateUpdate(intention,IDLE), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer,RESPONSE_TIME)),
                                        registry.reference("Idling")
                                        ),
                                        Controller.doAction(// SLOWER
                                                (rg,ds)-> List.of(new DataStateUpdate(intention,SLOWER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer,RESPONSE_TIME)),
                                                registry.reference("Idling")
                                        )
                                )
                        )
                )
        );

        registry.set("Moving_left",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Moving_left")),
                        Controller.ifThenElse(
                                DataState.equalsTo(other_lane, 0).and(DataState.equalsTo(my_position,-1)),
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,1), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                        registry.reference("Idling")
                                ),
                                Controller.doAction( // SLOWER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,1), new DataStateUpdate(my_timer, RESPONSE_TIME)),
                                        registry.reference("Idling")
                                )
                        )
                )
        );

        return new ExecController(registry.reference("Control"));

    }

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double my_new_acc;
        if(state.get(intention) == FASTER){ // Controller wants to do action FASTER
            my_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
        }else if(state.get(intention) == SLOWER){ // Controller wants to do action SLOWER
            my_new_acc = - Math.min(MAX_BRAKE, Math.max(MIN_BRAKE, MAX_BRAKE - rg.nextDouble() * SLOW_OFFSET));
        } else if(state.get(intention) == IDLE) { // Controller wants to do action IDLE
            my_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            my_new_acc = 0.0;
        }
        updates.add(new DataStateUpdate(my_acc, my_new_acc));

        double my_travel_x = (my_new_acc/2 + state.get(my_speed))*Math.cos((Math.PI/9)*state.get(my_move));
        double my_new_x = state.get(my_x) + my_travel_x;
        //double my_travel_y = (my_new_acc/2 + state.get(my_speed))*Math.sin((Math.PI/9)*state.get(my_move));
        double my_new_y = Math.min(8,Math.max(0,state.get(my_y) + (4/RESPONSE_TIME)*state.get(my_move)));
        double my_new_speed = Math.min(Math.max(0,state.get(my_speed) + my_new_acc), MAX_SPEED);
        updates.add(new DataStateUpdate(my_speed, my_new_speed));
        updates.add(new DataStateUpdate(my_x,my_new_x));
        updates.add(new DataStateUpdate(my_y,my_new_y));

        double other_new_acc;
        if (state.get(my_position)==1){
            if (state.get(dist) > state.get(safety_gap)){
                other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
            } else {
                double token = rg.nextDouble();
                if (token >= 0.5){
                    other_new_acc = MAX_ACCELERATION - rg.nextDouble() * MAX_ACCELERATION;
                } else {
                    other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                }
            }
        } else {
            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * MAX_ACCELERATION;
        }
        updates.add(new DataStateUpdate(other_acc,other_new_acc));

        double other_travel_x = (my_new_acc/2 + state.get(other_speed))*Math.cos((Math.PI/9)*state.get(other_move));
        double other_new_x = state.get(other_x) + other_travel_x;
        //double other_travel_y = (my_new_acc/2 + state.get(other_speed))*Math.sin((Math.PI/9)*state.get(other_move));
        double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + (4/RESPONSE_TIME)*state.get(other_move)));
        double other_new_speed = Math.min(Math.max(0,state.get(other_speed) + other_new_acc), MAX_SPEED);
        updates.add(new DataStateUpdate(other_speed, other_new_speed));
        updates.add(new DataStateUpdate(other_x,other_new_x));
        updates.add(new DataStateUpdate(other_y,other_new_y));

        double new_dist = Math.sqrt(Math.pow((other_new_x-my_new_x),2) + Math.pow((other_new_y-my_new_y),2));
        updates.add(new DataStateUpdate(dist, new_dist));

        double my_new_position = (my_new_x>= other_new_x)?1:-1;
        updates.add(new DataStateUpdate(my_position,my_new_position));

        double new_safety_gap = (my_new_position==-1)?calculateRSSSafetyDistance(RESPONSE_TIME, my_new_speed, other_new_speed):calculateRSSSafetyDistance(RESPONSE_TIME,other_new_speed,my_new_speed);
        updates.add(new DataStateUpdate(safety_gap, new_safety_gap));

        double my_new_timer = state.get(my_timer) - 1;
        updates.add(new DataStateUpdate(my_timer, my_new_timer));

        return updates;
    }



    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }


}

