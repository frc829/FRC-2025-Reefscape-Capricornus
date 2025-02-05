package frc.robot.commandfactories;

import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandDualIntake;


import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Value;

public class CommandDualIntakeFactory {
    private final CommandDualIntake commandDualIntake;

    public CommandDualIntakeFactory(CommandDualIntake commandDualIntake) {
        this.commandDualIntake = commandDualIntake;
        // TODO: call commandDualIntake's setDefaultCommand method and pass in a call to the moveAtVelocity method defined below.
        // TODO:  you'll passing in () -> 0.0, () -> 0.0 into the moveToVelocityMethod.
        // TODO: DEAD GUY NUMBER 1 SAYS HI SAYS HI

        commandDualIntake.setDefaultCommand(moveAtVelocity(() -> 0.0, () -> 0.0));


    }

    public Command moveAtVelocity(DoubleSupplier maxIntakeVelocity0Percentage, DoubleSupplier maxIntakeVelocity1Percentage) {
        MutDimensionless intakeVelocityPercent0 = Value.mutable(0.0);
        MutDimensionless intakeVelocityPercent1 = Value.mutable(0.0);
        IntakeWheelRequest.Velocity request0 = new IntakeWheelRequest.Velocity();
        IntakeWheelRequest.Velocity request1 = new IntakeWheelRequest.Velocity();
        request0.withVelocity(intakeVelocityPercent0);
        request1.withVelocity(intakeVelocityPercent1);
        Pair<IntakeWheelRequest, IntakeWheel> intakeRequests = new Pair<>(request0, request1);

          return commandDualIntake
                 .applyRequest(() -> {
                     intakeVelocityPercent0.mut_setMagnitude(maxIntakeVelocity0Percentage.getAsDouble());
                     intakeVelocityPercent0.mut_setMagnitude(maxIntakeVelocity1Percentage.getAsDouble());
                         request0.withVelocity
                     return intakeRequests;
                         //delete in post
                 });

    }

}
