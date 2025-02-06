package frc.robot.subsystems.wrist;

import digilib.wrist.WristRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

public class CommandWristFactory {
    private final CommandWrist commandWrist;

    public CommandWristFactory(CommandWrist commandWrist) {
        this.commandWrist = commandWrist;
        commandWrist.setDefaultCommand(hold());
    }

    public Command hold() {
        WristRequest.Hold request = new WristRequest.Hold();
//        return commandWrist.applyRequest(() -> request).withName("HOLD");
        return null;
    }

    public Command goToAngle(Angle position) {
        WristRequest.Position request = new WristRequest.Position();
        request.withPosition(position);
        return commandWrist.applyRequest(() -> request).withName(String.format("POSITION: %s degrees", position.in(Degrees)));
    }

    public Command moveAtVelocity(DoubleSupplier maxwristVelocityPercentage) {
        MutDimensionless wristVelocityPercent = Value.mutable(0.0);
        WristRequest.Velocity request = new WristRequest.Velocity();
        request.withVelocity(wristVelocityPercent);
        return commandWrist.applyRequest(() -> {
                     wristVelocityPercent.mut_setMagnitude(maxwristVelocityPercentage.getAsDouble());
                     return request;
                 });
    }

}
