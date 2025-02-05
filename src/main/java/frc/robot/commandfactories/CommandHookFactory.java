package frc.robot.commandfactories;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandHook;
import frc.robot.subsystems.CommandHook;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Value;

public class CommandHookFactory {
    private final CommandHook commandHook;

    public CommandHookFactory(CommandHook commandHook) {
        this.commandHook = commandHook;
        // TODO: call commandhooks's setDefaultCommand method and pass in a call to the hold method defined below.
        // NOTES:  hold() creates a hold command for the hook.  We then set it for the set in this todo.
    }

    public Command hold() {
        // TODO: create an hookRequest.Hold called request and assign a new hookRequest.Hold to it.
        // TODO: return command.applyRequest(() -> request).withName("HOLD")
        return null; // TODO: remove this when done.
    }

    public Command goToAngle(Angle position) {
        // TODO: create an hookRequest.Position called request.  Assign it appropriately.
        // TODO: called request.withPosition method and pass in position.
        // return commandhook.applyRequest(() -> request).withName(String.format("POSITION: %s degrees", position.in(Degrees)));
        return null; // TODO: remove this when done.
    }

    public Command moveAtVelocity(DoubleSupplier maxhookVelocityPercentage) {
        // TODO: create a MutDimensionless named hookVelocityPercent and assign Value.mutable(0.0);
        // TODO: hookRequest.Velocity called request.  Assign it appropriately.
        // TODO: call request withVelocity methodd and pass in hookVelocityPercent.
        // retu
        // TODO: uncomment this out.  return commandhook
        //         .applyRequest(() -> {
        //             hookVelocityPercent.mut_setMagnitude(maxhookVelocityPercentage.getAsDouble());
        //             return request;
        //         })
        return null; // TODO: remove this when done.
    }

}
