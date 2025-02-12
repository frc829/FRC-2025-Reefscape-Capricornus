package frc.robot.subsystems.dualIntake;

import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import java.awt.*;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Value;

public class CommandDualIntakeFactory {
    private final CommandDualIntake commandDualIntake;
    public final Trigger hasCoral;

    public CommandDualIntakeFactory(CommandDualIntake commandDualIntake) {
        this.commandDualIntake = commandDualIntake;
        this.hasCoral = commandDualIntake.hasCoral;
        commandDualIntake.setDefaultCommand(idle());
    }


    public Command idle() {
        IntakeWheelRequest.Idle request0 = new IntakeWheelRequest.Idle();
        IntakeWheelRequest.Idle request1 = new IntakeWheelRequest.Idle();
        Pair<IntakeWheelRequest, IntakeWheelRequest> request = new Pair<>(request0, request1);
        return commandDualIntake.applyRequest(() -> request).withName("INTAKE:IDLE");
    }

    public Command moveAtVelocity(Supplier<Dimensionless> maxIntakeVelocity0Percentage, Supplier<Dimensionless> maxIntakeVelocity1Percentage) {
        MutDimensionless intakeVelocityPercent0 = Value.mutable(0.0);
        MutDimensionless intakeVelocityPercent1 = Value.mutable(0.0);
        IntakeWheelRequest.Velocity request0 = new IntakeWheelRequest.Velocity();
        IntakeWheelRequest.Velocity request1 = new IntakeWheelRequest.Velocity();
        Pair<IntakeWheelRequest, IntakeWheelRequest> intakeRequests = new Pair<>(request0, request1);

        return commandDualIntake
                .applyRequest(() -> {
                    intakeVelocityPercent0.mut_replace(maxIntakeVelocity0Percentage.get());
                    intakeVelocityPercent1.mut_replace(maxIntakeVelocity1Percentage.get());
                    request0.withVelocity(intakeVelocityPercent0);
                    request1.withVelocity(intakeVelocityPercent1);
                    return intakeRequests;
                }).withName("INTAKE:VELOCITY");
    }
}
