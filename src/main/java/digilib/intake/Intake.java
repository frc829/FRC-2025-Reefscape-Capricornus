package digilib.intake;

import digilib.motorControllers.MotorController;

public class Intake {

    private final MotorController motorController;

    public Intake(MotorController motorController) {
        this.motorController = motorController;
    }

    public double getVoltageVolts() {
        return motorController.getVoltageVolts();
    }

    public double getCurrentAmps() {
        return motorController.getCurrentAmps();
    }

    public double getVelocityRadPerSecond() {
        return motorController.getVelocity();
    }

    public double getAccelerationRadPerSecondSq() {
        return motorController.getAcceleration();
    }

    public void applyVoltage(double voltageVolts) {
        motorController.applyVoltage(voltageVolts);
    }

    public void applyCurrent(double currentAmps) {
        motorController.applyCurrent(currentAmps);
    }

    public void applyVelocityUsingVoltage(double velocityRadPerSecond) {
        motorController.applyVelocityUsingVoltage(velocityRadPerSecond);
    }

    public void applyVelocityUsingCurrent(double velocityRadPerSecond) {
        motorController.applyVelocityUsingCurrent(velocityRadPerSecond);
    }
}
