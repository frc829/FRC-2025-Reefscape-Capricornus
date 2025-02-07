package digilib.controllers;

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
        // TODO: return a call to velocity.mut_setMagnitude() passing in getVelocityValue();
        return null; // TODO: remove this when done.
    }

    private double getVelocityValue() {
        // TODO: get x from a call to -MathUtil.applyDeadband() passing in controller.getLeftX() and deadband
        // TODO: repeat for y
        // TODO: get the velocity from Math.hypot() passing in x and y
        // TODO: return the minimum of velocity and 1 using Math.min
        return 0.0; // TODO: remove this when done.
    }

    public Dimensionless getRotationalVelocity() {
        // TODO: return a call to rotationalVelocity.mut_setMagnitude() passing in getRotationalVelocityValue();
        return null; // TODO: remove this when done.
    }

    private double getRotationalVelocityValue() {
        // TODO: get the leftTrigger value from a call to MathUtil.applyDeadband() passing in controller.getLeftTriggerAxis and deadband
        // TODO: repeat for rightTrigger
        // TODO: return the difference between leftTrigger and rightTrigger
        return 0.0; // TODO: remove this when done.
    }

    public Angle getHeading() {
        // TODO: return a call to heading.mut_setMagnitude() passing in getHeadingRadians();
        return null; // TODO: remove this when done.
    }

    private double getHeadingRadians() {
        // TODO: get x from a call to -MathUtil.applyDeadband() passing in controller.getLeftX() and deadband
        // TODO: repeat for y
        // TODO: return the heading from Math.atan() passing in x and y.  x is really the y and y is really the x
        return 0.0; // TODO: remove this when done.
    }

    public Angle getRotation() {
        // TODO: return a call to rotation.mut_setMagnitude() passing in getRotationRadians();
        return null; // TODO: remove this when done.
    }

    private double getRotationRadians() {
        // TODO: get x from a call to -MathUtil.applyDeadband() passing in controller.getLeftX() and deadband
        // TODO: repeat for y
        // TODO: return the rotation from Math.atan() passing in x and y.  x is really the y and y is really the x
        return 0.0; // TODO: remove this when done.
    }

    public Trigger getVelocityTrigger() {
        // TODO: return a new Trigger passing in a boolean supplier () -> getVelocityValue() != 0.0;
        return null; // TODO: remove this when done.
    }

    public Trigger getRobotCentricTrigger() {
        // TODO: God I love composing triggers instead of if statement logic.
        return getVelocityTrigger().and(getClockDriveTrigger().negate());
    }

    public Trigger getClockDriveTrigger() {
        // TODO: similar to getVelocityTrigger but the call is to getRotationRadians instead of getVelocityValue
        return null; // TODO: remove this when done.
    }
}
