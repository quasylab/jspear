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

package it.unicam.quasylab.jspear.examples.agriculturalDT;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ParallelController;
import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;
import java.util.stream.DoubleStream;

public class Main {

    public final static String[] VARIABLES =
            new String[] { "posX", "posY", "dirAngle", "steerAngle", "speed", "sensedSpeed",  "dirAngleNoise", "steerAngleNoise", "speedNoise"};
    public static final double L = 1.85;
    public static final double MAX_STEER_ANGLE = Math.PI/9;
    //public static final double MAX_DIR_ANGLE = Math.PI;
    //public static final double MIN_DIR_ANGLE = -Math.PI;
    public static final double MIN_SPEED = 0;
    public static final double MAX_SPEED = 3;
    public static final double MIN_ACC = 2;
    public static final double MAX_ACC = 1;
    public static final int TIME_OUT = 1;
    public static final double DIST_EPS = 0.1;
    public static final double DIR_EPS = 0.0;

    private static final int posX = 0;
    private static final int posY = 1;
    private static final int dirAngle = 2;
    private static final int steerAngle = 3;
    private static final int speed = 4;
    private static final int sensedSpeed = 5;
    private static final int dirAngleNoise = 6;
    public static final int steerAngleNoise = 7;
    private static final int speedNoise = 8;
    private static final int distance = 9;
    private static final int diffAngle = 10;
    private static final int timer = 11;
    private static final int NUMBER_OF_VARIABLES = 12;

    private static final double INIT_POSX = 0.0;
    private static final double INIT_POSY = 0.0;
    private static final double INIT_DIRANGLE = Math.PI/2;
    private static final double INIT_STEERANGLE = 0;
    private static final double INIT_SPEED = 0.0;
    private static final double INIT_ACC = 0.0;
    private static final double FINAL_POSX = 30.0;
    private static final double FINAL_POSY = 42.0;
    private static final double INIT_DIST = Math.sqrt((FINAL_POSX-INIT_POSX)*(FINAL_POSX-INIT_POSX) + (FINAL_POSY-INIT_POSY)*(FINAL_POSY-INIT_POSY));
    private static final double FINAL_DIRANGLE = Math.PI/3;
    private static final double FINAL_SPEED = 0.0;

    public static final double Kx = 0.15;     //parameters taken from VAML13
    public static final double Kd = 1/INIT_DIST;
    public static final double Kl = 1.8;
    public static final double Ko = 8;
    public static final double Kt = 0.01;
    public static final double KRT = 0.6;
    private static final int N = 100;



    public static void main(String[] args) throws IOException {
        try {
            RandomGenerator rand = new DefaultRandomGenerator();

            Controller controller = getController();
            DataState state = getInitialState(INIT_POSX,INIT_POSY,INIT_DIRANGLE,INIT_STEERANGLE,INIT_SPEED,INIT_ACC);
            ControlledSystem system = new ControlledSystem(controller, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, N);

            ArrayList<String> L = new ArrayList<>();
            L.add("x");
            L.add("y");
            L.add("theta");
            L.add("delta");
            L.add("speed");
            L.add("distance");
            L.add("diff_theta");
            L.add("ex");
            L.add("ey");
            L.add("eTheta");
            L.add("eRT");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(posX));
            F.add(ds->ds.get(posY));
            F.add(ds->ds.get(dirAngle));
            F.add(ds->ds.get(steerAngle));
            F.add(ds->ds.get(speed));
            F.add(ds->ds.get(distance));
            F.add(ds->ds.get(diffAngle));
            F.add(ds->Math.cos(ds.get(dirAngle))*(FINAL_POSX-ds.get(posX)) + Math.sin(ds.get(dirAngle))*(FINAL_POSY-ds.get(posY)));
            F.add(ds->-Math.sin(ds.get(dirAngle))*(FINAL_POSX-ds.get(posX)) + Math.cos(ds.get(dirAngle))*(FINAL_POSY-ds.get(posY)));
            F.add(ds->FINAL_DIRANGLE - ds.get(dirAngle));
            F.add(ds->FINAL_DIRANGLE - getThetaRT(ds.get(posX),ds.get(posY),ds.get(distance)));

            printLData(rand,L,F,system,500,1);


        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    public static double getThetaRT(double x, double y, double dist){
        if(dist > DIST_EPS){
            return Math.atan((FINAL_POSY-y)/(FINAL_POSX-x));
        } else {
            return FINAL_DIRANGLE;
        }
    }

    public static double[] evaluateDeltaZero(double x, double y, double theta, double dist) {
        double [] result = new double[2];
        double KD;
        if(dist > DIST_EPS){
            KD = 1/dist;
        } else {
            KD = Kd;
        }
        double ex = Math.cos(theta)*(FINAL_POSX-x) + Math.sin(theta)*(FINAL_POSY-y);
        double ey = -Math.sin(theta)*(FINAL_POSX-x) + Math.cos(theta)*(FINAL_POSY-y);
        double eTheta = FINAL_DIRANGLE - theta;
        double thetaRT = getThetaRT(x,y,dist);
        double eRT = FINAL_DIRANGLE - thetaRT;
        double centre;
        double v;
        double delta;
        if (eTheta>0) {
            centre = Kt * Math.tan(eTheta) + (KD * ey - Kl * dist * Math.sin(eRT) * Math.cos(eTheta)) / (Ko * Math.cos(eTheta)) + (KRT * Math.sin(eRT) * Math.sin(eRT) / (Math.sin(eTheta) * Math.cos(eTheta)));
        } else {
            centre = Kt * Math.tan(eTheta) + (KD * ey - Kl * dist * Math.sin(eRT) * Math.cos(eTheta)) / (Ko * Math.cos(eTheta)) + (KRT * Math.sin(eRT) * Math.sin(eRT) / (Math.sin(eRT) * Math.cos(eTheta)));
        }
        v = Math.min(MAX_SPEED, Math.max(MIN_SPEED, Kx * (KD * ex + Kl * dist * Math.sin(eRT) * Math.sin(eTheta) + Ko * Math.sin(eTheta) * centre)));
        delta = Math.min(MAX_STEER_ANGLE, Math.max(-MAX_STEER_ANGLE, Math.atan(L * centre)));
        result[0] = v;
        result[1] = delta;
        return result;
    }

    public static double[] evaluateDelta(double x, double y, double theta, double dist) {
        double [] result = new double[2];
        double ex = Math.cos(theta)*(FINAL_POSX-x) + Math.sin(theta)*(FINAL_POSY-y);
        double ey = -Math.sin(theta)*(FINAL_POSX-x) + Math.cos(theta)*(FINAL_POSY-y);
        double eTheta = FINAL_DIRANGLE - theta;
        double thetaRT = getThetaRT(x,y,dist);
        double eRT = FINAL_DIRANGLE - thetaRT;
        double centre = Kt*Math.tan(eTheta) + (Kd*ey - Kl*dist*Math.sin(eRT)*Math.cos(eTheta))/(Ko*Math.cos(eTheta)) + (KRT*Math.sin(eRT)*Math.sin(eRT)/(Math.sin(eTheta)*Math.cos(eTheta)));
        double v = FINAL_SPEED*Math.cos(eTheta) + Kx*(Kd*ex + Kl*dist*Math.sin(eRT)*Math.sin(eTheta) + Ko*Math.sin(eTheta)*centre);
        double delta = Math.min(MAX_STEER_ANGLE, Math.max(-MAX_STEER_ANGLE, Math.atan(L*centre)));
        result[0] = v;
        result[1] = delta;
        return result;
    }

    public static Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();
            registry.set("Ctrl",
                    Controller.ifThenElse(
                            DataState.greaterThan(distance, DIST_EPS).or(DataState.greaterThan(diffAngle,DIR_EPS)),
                            //Controller.ifThenElse(
                            //        DataState.greaterThan(distance, DIST_EPS).and(DataState.greaterThan(diffAngle,DIR_EPS)),
                            Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(speed, evaluateDeltaZero(ds.get(posX), ds.get(posY), ds.get(dirAngle), ds.get(distance))[0]),
                                            new DataStateUpdate(steerAngle, evaluateDeltaZero(ds.get(posX), ds.get(posY), ds.get(dirAngle), ds.get(distance))[1]),
                                            new DataStateUpdate(timer, TIME_OUT)),
                                    registry.reference("Idle")),
                            //        Controller.ifThenElse(
                            //                DataState.greaterThan(distance,DIST_EPS).and(DataState.lessOrEqualThan(diffAngle,DIR_EPS)),
                            //                Controller.doAction((rg,ds)->List.of( new DataStateUpdate(steerAngle,0), new DataStateUpdate(timer,TIME_OUT)), registry.reference("Idle")),
                            //                Controller.ifThenElse(
                            //                        DataState.lessOrEqualThan(distance,DIST_EPS).and(DataState.greaterThan(diffAngle, DIR_EPS)),
                            //                        Controller.doAction((rg,ds)->List.of(new DataStateUpdate(acc,-ds.get(acc)), new DataStateUpdate(steerAngle,ds.get(steerAngleErr)), new DataStateUpdate(timer,TIME_OUT)), registry.reference("Idle")),
                            //                        Controller.doAction((rg,ds)->List.of(new DataStateUpdate(acc,-MIN_ACC), new DataStateUpdate(steerAngle,0), new DataStateUpdate(timer,TIME_OUT)),registry.reference("Stop"))
                            //                )
                            //        )
                            //),
                            Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(speed, evaluateDeltaZero(ds.get(posX), ds.get(posY), ds.get(dirAngle), ds.get(distance))[0]),
                                                    new DataStateUpdate(steerAngle, evaluateDeltaZero(ds.get(posX), ds.get(posY), ds.get(dirAngle), ds.get(distance))[1]),
                                                    new DataStateUpdate(timer, TIME_OUT)),
                                            registry.reference("Stop"))
                                    //Controller.doAction((rg,ds)->List.of(new DataStateUpdate(acc,MAX_ACC/4), new DataStateUpdate(steerAngle,ds.get(steerAngleErr)), new DataStateUpdate(timer,TIME_OUT)),registry.reference("Idle")),

                    )
            );
            registry.set("Idle",
                    Controller.ifThenElse(
                            DataState.greaterThan(timer, 0),
                            Controller.doTick(registry.reference("Idle")),
                            registry.reference("Ctrl")
                    )
            );
            registry.set("Stop",
                    Controller.ifThenElse(
                            DataState.greaterThan(timer, 0),
                            Controller.doTick(registry.reference("Stop")),
                            Controller.ifThenElse(
                                    DataState.greaterThan(distance, DIST_EPS).or(DataState.greaterThan(diffAngle,DIR_EPS)),
                                    Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(speed, evaluateDeltaZero(ds.get(posX), ds.get(posY), ds.get(dirAngle), ds.get(distance))[0]),
                                                    new DataStateUpdate(steerAngle, evaluateDeltaZero(ds.get(posX), ds.get(posY), ds.get(dirAngle), ds.get(distance))[1]),
                                                    new DataStateUpdate(timer, TIME_OUT)),
                                            registry.reference("Idle")),
                                    Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(speed, 0),
                                                    new DataStateUpdate(steerAngle, 0),
                                                    new DataStateUpdate(timer, TIME_OUT)),
                                            registry.reference("Stop"))
                            )
                    )
            );

            return registry.reference("Ctrl");
    }

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        double x = state.get(posX);
        double y = state.get(posY);
        double theta = state.get(dirAngle);
        double delta = state.get(steerAngle);
        double v = state.get(speed);
        List<DataStateUpdate> updates = new LinkedList<>();
        double sensV = Math.min(MAX_SPEED, Math.max(MIN_SPEED, v + rg.nextDouble()*0.25-0.125));
        double newX = x + v*Math.cos(theta);
        double newY = y + v*Math.sin(theta);
        double newTheta = theta + Math.tan(delta)*v/L; // Math.min(MAX_DIR_ANGLE, Math.max(MIN_DIR_ANGLE, theta + Math.tan(delta)*v/L));
        //double newDelta = Math.min(MAX_STEER_ANGLE, Math.max(-MAX_STEER_ANGLE, Kt*(FINAL_DIRANGLE-newTheta) + Kp*(FINAL_POSY-newY)/(FINAL_POSX-newX)));
        //double newDelta = Math.min(MAX_STEER_ANGLE, Math.max(-MAX_STEER_ANGLE, Kt*(-newTheta) + Kp*(-newY)));
        //double newDist = Math.sqrt((FINAL_POSX-newX)*(FINAL_POSX-x) + (FINAL_POSY-newY)*(FINAL_POSY-newY)); // - (v * v + (MAX_ACC + MIN_ACC) * (MAX_ACC * TIME_OUT * TIME_OUT + 2 * Math.abs(v) * TIME_OUT)) / (2 * MIN_ACC);
        double newDist = Math.sqrt((FINAL_POSX-newX)*(FINAL_POSX-newX) + (FINAL_POSY-newY)*(FINAL_POSY-newY));
        updates.add(new DataStateUpdate(posX, newX));
        updates.add(new DataStateUpdate(posY, newY));
        updates.add(new DataStateUpdate(dirAngle, newTheta));
        updates.add(new DataStateUpdate(sensedSpeed,sensV));
        //updates.add(new DataStateUpdate(steerAngleErr, newDelta));
        //updates.add(new DataStateUpdate(speed,v));
        updates.add(new DataStateUpdate(distance,newDist));
        updates.add(new DataStateUpdate(diffAngle,Math.abs(FINAL_DIRANGLE - newTheta)));
        updates.add(new DataStateUpdate(timer,state.get(timer)-1));
        return updates;
    }

    public static DataState getInitialState(double x, double y, double theta, double delta, double v, double a) {
        Map<Integer, Double> values = new HashMap<>();
        values.put(posX, x);
        values.put(posY, y);
        values.put(dirAngle, theta);
        values.put(steerAngle, delta);
        values.put(speed, v);
        //values.put(acc, a);
        values.put(sensedSpeed,v);
        //values.put(steerAngleErr, Math.min(MAX_STEER_ANGLE, Math.max(-MAX_STEER_ANGLE, Kt*(FINAL_DIRANGLE-theta) + Kp*(FINAL_POSY-y)/(FINAL_POSX - x))));
        //values.put(steerAngleErr, Math.min(MAX_STEER_ANGLE, Math.max(-MAX_STEER_ANGLE, Kt*(-theta) + Kp*(-y))));
        //values.put(distance, Math.sqrt((FINAL_POSX-INIT_POSX)*(FINAL_POSX-INIT_POSX) + (FINAL_POSY-INIT_POSY)*(FINAL_POSY-INIT_POSY)));
        values.put(distance, Math.sqrt((FINAL_POSX-INIT_POSX)*(FINAL_POSX-INIT_POSX) + (FINAL_POSY-INIT_POSY)*(FINAL_POSY-INIT_POSY)));
        values.put(diffAngle, Math.abs(FINAL_DIRANGLE - INIT_DIRANGLE));
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, 0.0),0);
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

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

}
