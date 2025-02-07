package frc.robot.subsystems.pneumatics;

import digilib.claws.Claw;
import digilib.claws.ClawConstants;
import digilib.claws.ClawState;
import digilib.claws.SolenoidClaw;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.algaeClaw.CommandAlgaeClaw;
import frc.robot.subsystems.coralClaw.CommandCoralClaw;

import java.util.Map;

public class CommandPneumaticsConstants {

    private static final int module = 2;
    private static final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
    private static final PneumaticHub pneumaticHub = new PneumaticHub(module);

    private static class CommandAlgaeClawConstants {
        private static final int channel = 9;
        private static final String name = "Algae Claw";
        private static final ClawState.ClawValue startingState = ClawState.ClawValue.UNKNOWN;
        private static final Map<ClawState.ClawValue, Boolean> clawValueSolenoidMap = Map.of(
                ClawState.ClawValue.OPEN, true,
                ClawState.ClawValue.CLOSED, false
        );
        private static final ClawConstants constants = new ClawConstants(
                name,
                module,
                moduleType,
                clawValueSolenoidMap,
                startingState);
        private static final Solenoid solenoid = pneumaticHub.makeSolenoid(channel);
        private static final Claw claw = new SolenoidClaw(constants, solenoid);
    }

    private static class CommandCoralClawConstants {
        private static final int channel = 8;
        private static final String name = "Coral Claw";
        private static final ClawState.ClawValue startingState = ClawState.ClawValue.UNKNOWN;
        private static final Map<ClawState.ClawValue, Boolean> clawValueSolenoidMap = Map.of(
                ClawState.ClawValue.OPEN, true,
                ClawState.ClawValue.CLOSED, false
        );
        private static final ClawConstants constants = new ClawConstants(
                name,
                module,
                moduleType,
                clawValueSolenoidMap,
                startingState);
        private static final Solenoid solenoid = pneumaticHub.makeSolenoid(channel);
        private static final Claw claw = new SolenoidClaw(constants, solenoid);
    }

    public static CommandPneumatics createCommandPneumatics() {
        return new CommandPneumatics(pneumaticHub);
    }

    public static CommandAlgaeClaw createCommandAlgaeClaw() {
        return new CommandAlgaeClaw(CommandAlgaeClawConstants.claw);
    }

    public static CommandCoralClaw createCommandCoralClaw() {
        return new CommandCoralClaw(CommandCoralClawConstants.claw);
    }
}
