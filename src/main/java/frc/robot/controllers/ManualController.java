package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualController {

    private final double deadband;
    private final CommandXboxController controller;

    public ManualController(double deadband) {
        this.deadband = deadband;
        controller = new CommandXboxController(3);
    }

    public double getElevatorVelocity() {
        return -MathUtil.applyDeadband(controller.getLeftY(), deadband);
    }

    public Trigger elevator() {
        return new Trigger(() -> getElevatorVelocity() != 0.0);
    }

    public double getArmVelocity() {
        return -MathUtil.applyDeadband(controller.getRightY(), deadband);
    }

    public Trigger arm() {
        return new Trigger(() -> getArmVelocity() != 0);
    }

    public double getWristVelocity() {
        return MathUtil.applyDeadband(controller.getLeftTriggerAxis(), deadband)
                - MathUtil.applyDeadband(controller.getRightTriggerAxis(), deadband);
    }

    public Trigger wrist() {
        return new Trigger(() -> getWristVelocity() != 0);
    }

    public Trigger testElevatorPos() {
        return controller.a();
    }

    public Trigger testArmPos() {
        return controller.b();
    }

    public Trigger wristToggle() {
        return controller.y();
    }

    public Trigger algaeClawToggle() {
        return controller.leftBumper();
    }

    public Trigger coralClawToggle() {
        return controller.rightBumper();
    }

    public Trigger coralIn() {
        return controller.povDown();
    }

    public Trigger coralOut() {
        return controller.povUp();
    }

    public Trigger algaeIn() {
        return controller.povLeft();
    }

    public Trigger algaeOut() {
        return controller.povRight();
    }


}
