package frc.robot.subsystems.arm;

import digilib.arm.ArmRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutDimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Value;

public class CommandArmFactory {
    private final CommandArm commandArm;

    public CommandArmFactory(CommandArm commandArm) {
        this.commandArm = commandArm;
        commandArm.register();
        commandArm.setDefaultCommand(hold());
    }

    public Trigger atPosition(Angle position, Angle tolerance){
        return commandArm.atPosition(position, tolerance);
    }

    public Command hold() {
        ArmRequest.Hold request = new ArmRequest.Hold();
        return CommandArm.applyRequest(()-> request).withName("HOLD");
    }

    public Command goToAngle(Angle position) {
        ArmRequest.Position request = new ArmRequest.Position();
        request.withPosition(position);
    }

    public Command moveAtVelocity(DoubleSupplier maxArmVelocityPercentage) {
        MutDimensionless armVelocityPercent = Value.mutable(0.0);
        ArmRequest.Velocity request = new ArmRequest.Velocity();
        request.withVelocity(armVelocityPercent);
        return CommandArm.applyRequest(() -> {
            armVelocityPercent.mut_setMagnitude(maxArmVelocityPercentage.getAsDouble());
            return request;
                });

    }

}
