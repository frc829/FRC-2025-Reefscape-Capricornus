package frc.robot.subsystems.elevator;

import digilib.elevator.Elevator;
import digilib.elevator.ElevatorRequest;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Value;

public class CommandElevatorFactory {
    private final CommandElevator commandElevator;

    public CommandElevatorFactory(CommandElevator commandElevator) {
        this.commandElevator = commandElevator;
        commandElevator.setDefaultCommand(hold());
    }

    public Command hold() {
        ElevatorRequest.Hold request =  new ElevatorRequest.Hold();
        return commandElevator.applyRequest(() -> request).withName("HOLD");

    }

    public Command goToPosition(Distance position) {
        ElevatorRequest.Position request =  new ElevatorRequest.Position();
        request.withPosition(position);
        return commandElevator.applyRequest(() -> request).withName(String.format("POSITION: %s meters", position.in(Meters)));

    }

    public Command moveAtVelocity(DoubleSupplier maxElevatorVelocityPercentage) {
        MutDimensionless elevatorVelocityPercent = Value.mutable(0.0);
        ElevatorRequest.Velocity request =  new ElevatorRequest.Velocity();
        request.withVelocity(elevatorVelocityPercent);
        return commandElevator
                 .applyRequest(() -> {
                     elevatorVelocityPercent.mut_setMagnitude(maxElevatorVelocityPercentage.getAsDouble());
                     return request;
                 });
    }

}
