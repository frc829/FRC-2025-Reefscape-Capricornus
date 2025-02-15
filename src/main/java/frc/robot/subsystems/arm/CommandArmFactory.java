package frc.robot.subsystems.arm;

import digilib.arm.ArmRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class CommandArmFactory {
    private final CommandArm commandArm;

    public CommandArmFactory(CommandArm commandArm) {
        this.commandArm = commandArm;
        commandArm.setDefaultCommand(commandArm.hold());
    }

    public Trigger atPosition(Angle position, Angle tolerance) {
        return commandArm.atPosition(position, tolerance);
    }



    public Command goToAngle(Angle position) {
        ArmRequest.Position request = new ArmRequest.Position();
        request.withPosition(position.in(Radians));
        return commandArm.applyRequest(() -> request).withName(String.format("ARM:%s degrees", position.in(Degrees)));
    }

    public Command moveAtVelocity(DoubleSupplier value) {
        ArmRequest.Velocity request = new ArmRequest.Velocity();
        return commandArm.applyRequest(() -> request.withVelocity(value.getAsDouble())).withName("ARM:VELOCITY");
    }
}