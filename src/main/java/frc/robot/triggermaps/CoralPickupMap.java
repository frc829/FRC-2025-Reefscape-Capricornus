package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralPickup;

public class CoralPickupMap {

    private final CommandXboxController controller;
    private final CoralPickup coral;

    public CoralPickupMap(CommandXboxController controller,
                          CoralPickup coral) {
        this.controller = controller;
        this.coral = coral;

        bindCoralFloorPickup();
        bindCoralStationPickup();
        bindCoralHold();
        bindCoralStationBackPickup();
    }

    private void bindCoralFloorPickup() {
        controller.rightBumper()
                .whileTrue(coral.floor())
                .onFalse(coral.hold());
    }

    private void bindCoralStationPickup() {
        controller.povUp()
                .whileTrue(coral.station())
                .onFalse(coral.hold());
    }

    private void bindCoralHold() {
        controller.back()
                .onTrue(coral.hardReset());
    }

    private void bindCoralStationBackPickup() {
        controller.leftBumper()
                .whileTrue(coral.stationBack())
                .onFalse(coral.holdFromBack());
    }
}
