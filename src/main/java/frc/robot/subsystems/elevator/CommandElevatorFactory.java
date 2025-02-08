package frc.robot.subsystems.elevator;

import digilib.elevator.ElevatorRequest;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class CommandElevatorFactory {
    private final CommandElevator commandElevator;

    public CommandElevatorFactory(CommandElevator commandElevator) {
        this.commandElevator = commandElevator;
        commandElevator.setDefaultCommand(hold());
    }

    public Trigger atPosition(Distance position, Distance tolerance){
        return commandElevator.atPosition(position, tolerance);
    }

    public Command hold() {
        ElevatorRequest.Hold request = new ElevatorRequest.Hold();
        return commandElevator.applyRequest(() -> request).withName("ELEVATOR:HOLD");
    }

    public Command goToPosition(Distance position) {
        ElevatorRequest.Position request = new ElevatorRequest.Position();
        request.withPosition(position);
        return commandElevator.applyRequest(() -> request).withName(String.format("ELEVATOR:%s meters", position.in(Meters)));
    }

    public Command moveAtVelocity(Supplier<Dimensionless> maxElevatorVelocityPercentage) {
        MutDimensionless elevatorVelocityPercent = Value.mutable(0.0);
        ElevatorRequest.Velocity request = new ElevatorRequest.Velocity();
        request.withVelocity(elevatorVelocityPercent);
        return commandElevator.applyRequest(() -> {
            elevatorVelocityPercent.mut_replace(maxElevatorVelocityPercentage.get());
            return request.withVelocity(elevatorVelocityPercent);
        }).withName("ELEVATOR:VELOCITY");
    }

}
