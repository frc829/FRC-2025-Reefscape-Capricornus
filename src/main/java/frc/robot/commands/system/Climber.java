package frc.robot.commands.system;

import digilib.winch.WinchRequest;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.winch.WinchSubsystem;

import java.util.function.Supplier;

public class Climber {

    private final WinchSubsystem winch;

    public Climber(WinchSubsystem winch) {
        this.winch = winch;
    }

    public Command climb(Supplier<Dimensionless> dutyCycle) {
        return climbAtDutyCycle(dutyCycle);
    }

    public Command climbAtDutyCycle(Supplier<Dimensionless> dutyCycle) {
        WinchRequest.DutyCycle request = new WinchRequest.DutyCycle();
        return winch.applyRequest(() -> request.withDutyCycle(dutyCycle.get()));
    }
}
