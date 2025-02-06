package nl.tue.ABZ2025;

import Scenarios.SingleLaneTwoCars;
import Scenarios.TwoLanesTwoCars;

import java.io.IOException;

public class Main {
    public static void main(String[] args) throws IOException {
        try{
            new TwoLanesTwoCars();
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }
}