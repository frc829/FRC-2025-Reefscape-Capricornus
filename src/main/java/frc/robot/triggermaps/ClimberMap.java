package frc.robot.triggermaps;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberMap {

    private static final double maxVoltage = 12.0;

    private final CommandJoystick controller;
    private final double deadband;
    private final ClimberSubsystem climber;


    public ClimberMap(CommandJoystick controller,
                      double deadband,
                      ClimberSubsystem climber) {
        this.controller = controller;
        this.deadband = deadband;
        this.climber = climber;

        bindClimb();

    }

    private void bindClimb() {
        controller.axisMagnitudeGreaterThan(1, deadband)
                .whileTrue(climber.toVoltage(getClimbVoltageScalar()));
    }

    private double getClimbVoltageScalar() {
        return maxVoltage * MathUtil.applyDeadband(controller.getY(), deadband);
    }
}
