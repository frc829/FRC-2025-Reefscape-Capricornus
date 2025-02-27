package frc.robot.triggermaps;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.game.CoralPickup;

import static edu.wpi.first.wpilibj2.command.Commands.idle;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
    }

    private void bindCoralFloorPickup() {
        controller.rightBumper()
                .whileTrue(
                        sequence(coral.coralFloor(), idle())
                                .withName("Pickup: Coral Floor"))
                .onFalse(coral.coralHold());
    }

    private void bindCoralStationPickup() {
        controller.povUp()
                .whileTrue(
                        sequence(coral.coralStation(), idle())
                                .withName("Pickup: Coral Station"))
                .onFalse(coral.coralHold());
    }

    private void bindCoralHold(){
        controller.back().onTrue(coral.coralHold());
    }
}
