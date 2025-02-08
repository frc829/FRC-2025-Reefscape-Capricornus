package digilib.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Value;

public class ManualController {

    private final double deadband;
    private final CommandXboxController controller;
    private final MutDimensionless elevatorVelocity = Value.mutable(0.0);
    private final MutDimensionless armVelocity = Value.mutable(0.0);

    public ManualController(double deadband) {
        this.deadband = deadband;
        controller = new CommandXboxController(3);
    }

    public Dimensionless getElevatorVelocity() {
        return elevatorVelocity.mut_setMagnitude(getElevatorVelocityValue());
    }

    private double getElevatorVelocityValue() {
        return -MathUtil.applyDeadband(controller.getLeftY(), deadband);
    }

    public Trigger elevator() {
        return new Trigger(() -> getElevatorVelocityValue() != 0.0);
    }

    public Dimensionless getArmVelocity() {
        return armVelocity.mut_setMagnitude(getArmVelocityValue());
    }

    private double getArmVelocityValue() {
        return -MathUtil.applyDeadband(controller.getRightY(), deadband);

    }

    public Trigger arm() {
        return new Trigger(() -> getArmVelocityValue() != 0);
    }


}
