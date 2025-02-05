package frc.robot.commandfactories;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandElevator;

import java.util.function.DoubleSupplier;

public class CommandElevatorFactory {
    private final CommandElevator commandElevator;

    public CommandElevatorFactory(CommandElevator commandElevator) {
        this.commandElevator = commandElevator;
        // TODO: call commandElevator's setDefaultCommand method and pass in a call to the hold method defined below.
        // NOTES:  hold() creates a hold command for the elevator.  We then set it for the set in this todo.
    }

    public Command hold() {
        // TODO: create an elevatorRequest.Hold called request and assign a new elevatorRequest.Hold to it.  
        // TODO: return command.applyRequest(() -> request).withName("HOLD")
        return null; // TODO: remove this when done.
    }

    public Command goToPosition(Distance position) {
        // TODO: create an elevatorRequest.Position called request.  Assign it appropriately.
        // TODO: called request.withPosition method and pass in position.  
        // return commandElevator.applyRequest(() -> request).withName(String.format("POSITION: %s meters", position.in(Meters)));
        return null; // TODO: remove this when done.
    }

    public Command moveAtVelocity(DoubleSupplier maxElevatorVelocityPercentage) {
        // TODO: create a MutDimensionless named elevatorVelocityPercent and assign Value.mutable(0.0);
        // TODO: elevatorRequest.Velocity called request.  Assign it appropriately.  
        // TODO: call request withVelocity method and pass in elevatorVelocityPercent.
        // return
        // TODO: uncomment this out.  return commandElevator
        //         .applyRequest(() -> {
        //             elevatorVelocityPercent.mut_setMagnitude(maxElevatorVelocityPercentage.getAsDouble());
        //             return request;
        //         })
        return null; // TODO: remove this when done.
    }

}
