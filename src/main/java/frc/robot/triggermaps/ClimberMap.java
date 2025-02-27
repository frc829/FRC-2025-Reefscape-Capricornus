package frc.robot.triggermaps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.system.Climber;

import static edu.wpi.first.units.Units.Value;

public class ClimberMap {

    private final CommandJoystick controller;
    private final double deadband;
    private final Climber climber;
    private final MutDimensionless climbDutyCyclePercent = Value.mutable(0.0);


    public ClimberMap(CommandJoystick controller,
                      double deadband,
                      Climber climber) {
        this.controller = controller;
        this.deadband = deadband;
        this.climber = climber;
    }

    private Dimensionless getClimbDutyCycle() {
        return climbDutyCyclePercent.mut_setBaseUnitMagnitude(getClimbDutyCycleValue());
    }

    private double getClimbDutyCycleValue() {
        return MathUtil.applyDeadband(controller.getY(), deadband);
    }
}
