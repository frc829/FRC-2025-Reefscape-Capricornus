package digilib.gearbox;

import digilib.linearsystem.Motor;

public class Gearbox {

    private final Motor motor;
    private final int numMotors;
    private final double gearing;

    public Gearbox(Motor motor, int numMotors, double gearing) {
        this.motor = motor;
        this.numMotors = numMotors;
        this.gearing = gearing;
    }



}
