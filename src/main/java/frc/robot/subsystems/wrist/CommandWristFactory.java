package frc.robot.subsystems.wrist;

import digilib.wrist.WristRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

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
        request.withPosition(position.in(Radians));
        return commandWrist.applyRequest(() -> request).withName(String.format("WRIST:%s degrees", position.in(Degrees)));
    }

    public Command moveAtVelocity(DoubleSupplier value) {
        WristRequest.Velocity request = new WristRequest.Velocity();
        return commandWrist.applyRequest(() -> request.withVelocity(value.getAsDouble())).withName("WRIST:VELOCITY");
    }
}
