package nl.tue.ABZ2025;

import Scenarios.AIMutipleLanes;

public class Main {
    public static void main(String[] args) {
        double[] sensorPertubationOffsets = new double[]{0.25};
        double[] invisibilityChances = new double[]{0.25};
        for(int i = 0; i < sensorPertubationOffsets.length; i++){
            new AIMutipleLanes(sensorPertubationOffsets[i], invisibilityChances[i]);
        }
    }
}