package frc.robot;

import digilib.claws.ClawState.ClawValue;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static digilib.claws.ClawState.ClawValue.UNKNOWN;

public class ManipulatorState {
    private double armAngleDegrees = 0.0;
    private double elevatorHeightMeters = 0.0;
    private double wristAngleDegrees = 0.0;
    private double algaeIntakeSpeed = 0.0;
    private double coralIntakeSpeed = 0.0;
    private ClawValue algaeClawValue = UNKNOWN;
    private ClawValue coralClawValue = UNKNOWN;
    private Trigger endingCondition;


    public double getArmAngleDegrees() {
        return armAngleDegrees;
    }

    public ManipulatorState withArmAngleDegrees(double armAngleDegrees) {
        this.armAngleDegrees = armAngleDegrees;
        return this;
    }

    public double getElevatorHeightMeters() {
        return elevatorHeightMeters;
    }

    public ManipulatorState withElevatorHeightMeters(double elevatorHeightMeters) {
        this.elevatorHeightMeters = elevatorHeightMeters;
        return this;
    }

    public double getWristAngleDegrees() {
        return wristAngleDegrees;
    }

    public ManipulatorState withWristAngleDegrees(double wristAngleDegrees) {
        this.wristAngleDegrees = wristAngleDegrees;
        return this;
    }

    public double getAlgaeIntakeSpeed() {
        return algaeIntakeSpeed;
    }

    public ManipulatorState withAlgaeIntakeSpeed(double algaeIntakeSpeed) {
        this.algaeIntakeSpeed = algaeIntakeSpeed;
        return this;
    }

    public double getCoralIntakeSpeed() {
        return coralIntakeSpeed;
    }

    public ManipulatorState withCoralIntakeSpeed(double coralIntakeSpeed) {
        this.coralIntakeSpeed = coralIntakeSpeed;
        return this;
    }

    public ClawValue getAlgaeClawValue() {
        return algaeClawValue;
    }

    public ManipulatorState withAlgaeClawValue(ClawValue algaeClawValue) {
        this.algaeClawValue = algaeClawValue;
        return this;
    }

    public ClawValue getCoralClawValue() {
        return coralClawValue;
    }

    public ManipulatorState withCoralClawValue(ClawValue coralClawValue) {
        this.coralClawValue = coralClawValue;
        return this;
    }

    public Trigger getEndingCondition(){
        return endingCondition;
    }

    public ManipulatorState withEndingCondition(Trigger endingCondition) {
        this.endingCondition = endingCondition;
        return this;
    }
}
