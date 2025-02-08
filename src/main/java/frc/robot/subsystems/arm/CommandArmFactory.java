package frc.robot.subsystems.arm;

import digilib.arm.ArmRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

public class CommandArmFactory {
    private final CommandArm commandArm;

    public CommandArmFactory(CommandArm commandArm) {
        this.commandArm = commandArm;
        commandArm.register();
        commandArm.setDefaultCommand(hold());
    }

    public Trigger atPosition(Angle position, Angle tolerance) {
        return commandArm.atPosition(position, tolerance);
    }

    public Command hold() {
        ArmRequest.Hold request = new ArmRequest.Hold();
        return commandArm.applyRequest(() -> request).withName("ARM:HOLD");
    }

    public Command goToAngle(Angle position) {
        ArmRequest.Position request = new ArmRequest.Position();
        request.withPosition(position);
        return commandArm.applyRequest(() -> request).withName(String.format("ARM:%s degrees", position.in(Degrees)));

    }

    public Command moveAtVelocity(Supplier<Dimensionless> maxArmVelocityPercentage) {
        MutDimensionless armVelocityPercent = Value.mutable(0.0);
        ArmRequest.Velocity request = new ArmRequest.Velocity();
        request.withVelocity(armVelocityPercent);
        return commandArm.applyRequest(() -> {
            armVelocityPercent.mut_replace(maxArmVelocityPercentage.get());
            return request.withVelocity(armVelocityPercent);
        }).withName("ARM:VELOCITY");
    }
}