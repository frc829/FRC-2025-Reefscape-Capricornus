package frc.robot.commandfactories;

import digilib.intakeWheel.IntakeWheel;
import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandDualIntake;
import frc.robot.subsystems.CommandDualIntake;

import java.util.function.DoubleSupplier;

public class CommandDualIntakeFactory {
    private final CommandDualIntake commandDualIntake;

    public CommandDualIntakeFactory(CommandDualIntake commandDualIntake) {
        this.commandDualIntake = commandDualIntake;
        // TODO: call commandintake's setDefaultCommand method and pass in a call to the hold method defined below.
        // NOTES:  hold() creates a hold command for the intake.  We then set it for the set in this todo.
    }

    public Command moveAtVelocity(DoubleSupplier maxIntakeVelocity0Percentage, DoubleSupplier maxIntakeVelocity1Percentage) {
        // TODO: create a MutDimensionless named intakeVelocityPercent0 and assign Value.mutable(0.0);
        // TODO: create a MutDimensionless named intakeVelocityPercent1 and assign Value.mutable(0.0);
        // TODO: intakeRequest.Velocity called request0.  Assign it appropriately.
        // TODO: intakeRequest.Velocity called request1.  Assign it appropriately.
        // TODO: call request0 withVelocity method and pass in intakeVelocityPercent0.
        // TODO: call request1 withVelocity method and pass in intakeVelocityPercent1.
        //  TODO: uncomment this:  Pair<IntakeWheelRequest, IntakeWheel> intakeRequests = new Pair<>(request0, request1);
        // return
        // TODO: uncomment this out.  return commandIntake
        //         .applyRequest(() -> {
        //             intakeVelocityPercent0.mut_setMagnitude(maxIntakeVelocity0Percentage.getAsDouble());
        //             intakeVelocityPercent0.mut_setMagnitude(maxIntakeVelocity1Percentage.getAsDouble());
        //                 request0.withVelocity
        //             return intakeRequests;
        //         })
        return null; // TODO: remove this when done.
    }

}
