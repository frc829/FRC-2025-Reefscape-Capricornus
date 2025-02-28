package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.game.AlgaePickup;

import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;

public class AlgaePickupMap {

    private final CommandXboxController controller;
    private final double deadband;
    private final AlgaePickup algae;

    public AlgaePickupMap(CommandXboxController controller,
                          double deadband,
                          AlgaePickup algae) {
        this.controller = controller;
        this.deadband = deadband;
        this.algae = algae;

        bindAlgaeFloorPickup();
        bindAlgaeL2Pickup();
        bindAlgaeL3Pickup();
    }

    private void bindAlgaeFloorPickup() {
        controller.axisMagnitudeGreaterThan(kRightTrigger.value, deadband)
                .whileTrue(algae.floor())
                .onFalse(algae.hold());
    }

    private void bindAlgaeL2Pickup() {
        controller.povRight()
                .whileTrue(algae.L2())
                .onFalse(algae.hold());
    }

    private void bindAlgaeL3Pickup() {
        controller.leftStick()
                .whileTrue(algae.L3())
                .onFalse(algae.hold());
    }
}
