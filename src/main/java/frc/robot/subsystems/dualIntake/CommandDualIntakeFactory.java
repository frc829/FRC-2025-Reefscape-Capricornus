package frc.robot.subsystems.dualIntake;

import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import java.awt.*;
import java.util.function.DoubleSupplier;
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

    public Command moveAtVelocity(DoubleSupplier maxIntak0VelocityValue, DoubleSupplier maxIntak1VelocityValue) {
        IntakeWheelRequest.Velocity request0 = new IntakeWheelRequest.Velocity();
        IntakeWheelRequest.Velocity request1 = new IntakeWheelRequest.Velocity();
        Pair<IntakeWheelRequest, IntakeWheelRequest> intakeRequests = new Pair<>(request0, request1);

        return commandDualIntake
                .applyRequest(() -> {
                    request0.withVelocity(maxIntak0VelocityValue.getAsDouble());
                    request1.withVelocity(maxIntak1VelocityValue.getAsDouble());
                    return intakeRequests;
                }).withName("INTAKE:VELOCITY");
    }
}
