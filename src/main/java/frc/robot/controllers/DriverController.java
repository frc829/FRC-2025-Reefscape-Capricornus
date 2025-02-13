package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Value;

public class DriverController {

    private final double deadband;
    private final CommandXboxController controller;
    private final MutDimensionless velocity = Value.mutable(0.0);
    private final MutDimensionless rotationalVelocity = Value.mutable(0.0);
    private final MutAngle heading = Radians.mutable(0.0);
    private final MutAngle rotation = Radians.mutable(0.0);

    public DriverController(double deadband) {
        this.deadband = deadband;
        controller = new CommandXboxController(0);
    }

    public Dimensionless getVelocity() {
        return velocity.mut_setMagnitude(getVelocityValue());
    }

    private double getVelocityValue() {
        double x = -MathUtil.applyDeadband(controller.getLeftX(), deadband);
        double y = -MathUtil.applyDeadband(controller.getLeftY(), deadband);
        double velocity = Math.hypot(x, y);
        return Math.min(velocity, 1);
    }

    public Dimensionless getRotationalVelocity() {
        return rotationalVelocity.mut_setMagnitude(getRotationalVelocityValue());
    }

    private double getRotationalVelocityValue() {
        double leftTrigger = MathUtil.applyDeadband(controller.getLeftTriggerAxis(), deadband);
        double rightTrigger = MathUtil.applyDeadband(controller.getRightTriggerAxis(), deadband);
        return leftTrigger - rightTrigger;
    }

    public Angle getHeading() {
        return heading.mut_setMagnitude(getHeadingRadians());
    }

    private double getHeadingRadians() {
        double x = -MathUtil.applyDeadband(controller.getLeftX(), deadband);
        double y = -MathUtil.applyDeadband(controller.getLeftY(), deadband);
        return Math.atan2(y, x);
    }

    public Angle getRotation() {
        return rotation.mut_setMagnitude(getRotationRadians());
    }

    private double getRotationRadians() {
        double x = -MathUtil.applyDeadband(controller.getRightX(), deadband);
        double y = -MathUtil.applyDeadband(controller.getRightY(), deadband);
        return Math.atan2(y, x);
    }

    public Trigger getVelocityTrigger() {
        return new Trigger(() -> getVelocityValue() != 0.0);
    }

    public Trigger getRobotCentricTrigger() {
        return getVelocityTrigger().and(getClockDriveTrigger().negate());
    }

    public Trigger getClockDriveTrigger() {
        return new Trigger(() -> getClockDriveTriggerValue() != 0.0);
    }

    private double getClockDriveTriggerValue() {
        double x = MathUtil.applyDeadband(controller.getRightX(), deadband);
        double y = MathUtil.applyDeadband(controller.getRightY(), deadband);
        return Math.hypot(x, y);
    }
}
