package frc.robot.commandfactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandWrist;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

public class CommandWristFactory {
    private final CommandWrist commandWrist;

    public CommandWristFactory(CommandWrist commandWrist) {
        this.commandWrist = commandWrist;
        // TODO: call commandWrist's setDefaultCommand method and pass in a call to the hold method defined below.  
        // NOTES:  hold() creates a hold command for the wrist.  We then set it for the set in this todo.
    }

    public Command hold() {
        // TODO: create an wristRequest.Hold called request and assign a new wristRequest.Hold to it.  
        // TODO: return command.applyRequest(() -> request).withName("HOLD")
        return null; // TODO: remove this when done.
    }

    public Command goToAngle(Angle position) {
        // TODO: create an wristRequest.Position called request.  Assign it appropriately.
        // TODO: called request.withPosition method and pass in position.  
        // return commandWrist.applyRequest(() -> request).withName(String.format("POSITION: %s degrees", position.in(Degrees)));
        return null; // TODO: remove this when done.
    }

    public Command moveAtVelocity(DoubleSupplier maxwristVelocityPercentage) {
        // TODO: create a MutDimensionless named wristVelocityPercent and assign Value.mutable(0.0);
        // TODO: wristRequest.Velocity called request.  Assign it appropriately.  
        // TODO: call request withVelocity methodd and pass in wristVelocityPercent.
        // return
        // TODO: uncomment this out.  return commandWrist
        //         .applyRequest(() -> {
        //             wristVelocityPercent.mut_setMagnitude(maxwristVelocityPercentage.getAsDouble());
        //             return request;
        //         })
        return null; // TODO: remove this when done.
    }

}
