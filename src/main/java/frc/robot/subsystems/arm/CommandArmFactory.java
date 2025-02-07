package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

public class CommandArmFactory {
    private final CommandArm commandArm;

    public CommandArmFactory(CommandArm commandArm) {
        this.commandArm = commandArm;
        commandArm.register();
        // TODO: call commandArms's setDefaultCommand method and pass in a call to the hold method defined below.
        // NOTES:  hold() creates a hold command for the arm.  We then set it for the set in this todo.
    }

    public Trigger atPosition(Angle position, Angle tolerance){
        return commandArm.atPosition(position, tolerance);
    }

    public Command hold() {
        // TODO: create an ArmRequest.Hold called request and assign a new ArmRequest.Hold to it.
        // TODO: return command.applyRequest(() -> request).withName("HOLD")
        return null; // TODO: remove this when done.
    }

    public Command goToAngle(Angle position) {
        // TODO: create an ArmRequest.Position called request.  Assign it appropriately.
        // TODO: called request.withPosition method and pass in position.
        // return commandArm.applyRequest(() -> request).withName(String.format("POSITION: %s degrees", position.in(Degrees)));
        return null; // TODO: remove this when done.
    }

    public Command moveAtVelocity(DoubleSupplier maxArmVelocityPercentage) {
        // TODO: create a MutDimensionless named armVelocityPercent and assign Value.mutable(0.0);
        // TODO: ArmRequest.Velocity called request.  Assign it appropriately.
        // TODO: call request withVelocity methodd and pass in armVelocityPercent.
        // retu
        // TODO: uncomment this out.  return commandArm
        //         .applyRequest(() -> {
        //             armVelocityPercent.mut_setMagnitude(maxArmVelocityPercentage.getAsDouble());
        //             return request;
        //         })
        return null; // TODO: remove this when done.
    }

}
