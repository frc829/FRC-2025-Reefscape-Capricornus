package frc.robot.subsystems.coralClaw;

import digilib.claws.Claw;
import digilib.claws.ClawConstants;
import digilib.claws.ClawValue;
import digilib.claws.SolenoidClaw;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import static digilib.claws.ClawValue.*;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
import static frc.robot.subsystems.coralClaw.CommandCoralClawConstants.Control.solenoidOnClawValue;

public class CommandCoralClawConstants {

    static final class Control {
        static final ClawValue solenoidOnClawValue = OPEN;
    }

    static final class Mechanism {
        static final String name = "Claw: Coral";
        static final PneumaticsModuleType moduleType = REVPH;
        static final ClawConstants constants = new ClawConstants(
                name,
                moduleType,
                solenoidOnClawValue);
    }

    static final class SolenoidConstants {
        static final int channel = 9;
    }

    public static CommandCoralClaw create(PneumaticsBase pneumaticsBase) {
        Solenoid solenoid = pneumaticsBase.makeSolenoid(SolenoidConstants.channel);
        Claw claw = new SolenoidClaw(Mechanism.constants, solenoid);
        return new CommandCoralClaw(claw);
    }
}
