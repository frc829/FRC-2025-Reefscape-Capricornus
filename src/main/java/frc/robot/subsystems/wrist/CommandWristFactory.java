package frc.robot.subsystems.wrist;

import digilib.wrist.WristRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

public class CommandWristFactory {
    private final CommandWrist commandWrist;

    public CommandWristFactory(CommandWrist commandWrist) {
        this.commandWrist = commandWrist;
        commandWrist.setDefaultCommand(hold());
    }

    public Trigger atPosition(Angle position, Angle tolerance){
        return commandWrist.atPosition(position, tolerance);
    }

    public Command hold() {
        WristRequest.Hold request = new WristRequest.Hold();
        return commandWrist.applyRequest(() -> request).withName("WRIST:HOLD");
    }

    public Command goToAngle(Angle position) {
        WristRequest.Position request = new WristRequest.Position();
        request.withPosition(position);
        return commandWrist.applyRequest(() -> request).withName(String.format("WRIST:%s degrees", position.in(Degrees)));
    }

    public Command moveAtVelocity(Supplier<Dimensionless> maxWristVelocityPercentage) {
        MutDimensionless wristVelocityPercent = Value.mutable(0.0);
        WristRequest.Velocity request = new WristRequest.Velocity();
        request.withVelocity(wristVelocityPercent);
        return commandWrist.applyRequest(() -> {
            wristVelocityPercent.mut_replace(maxWristVelocityPercentage.get());
            return request.withVelocity(wristVelocityPercent);
        }).withName("WRIST:VELOCITY");
    }
}
