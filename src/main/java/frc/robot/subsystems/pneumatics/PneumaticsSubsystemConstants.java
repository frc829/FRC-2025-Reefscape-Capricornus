package frc.robot.subsystems.pneumatics;

import digilib.claws.Claw;
import digilib.claws.SolenoidClaw;
import digilib.pneumatics.Pneumatics;
import digilib.pneumatics.REVPneumatics;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.claws.Claw.*;
import static digilib.claws.Claw.Value.OPEN;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;

public class PneumaticsSubsystemConstants {

    static final class Module {

        static final class Mechanism {
            static final String name = "Pneumatics";
        }

        static final class PneumaticsModule {
            static final int module = 2;
            static final PneumaticHub pneumaticHub = new PneumaticHub(module);
            static final Pneumatics.Config constants = new Pneumatics.Config(Mechanism.name);
        }

    }

    static final class AlgaeClaw {

        static final class Control {
            static final Value solenoidOnClawValue = OPEN;
        }

        static final class Mechanism {
            static final String name = "Claw: Algae";
            static final PneumaticsModuleType moduleType = REVPH;
            static final Claw.Config constants = new Claw.Config(
                    name,
                    moduleType,
                    AlgaeClaw.Control.solenoidOnClawValue);
        }

        static final class SolenoidConstants {
            static final int channel = 9;
        }

    }

    static class CoralClaw {

        static final class Control {
            static final Value solenoidOnClawValue = OPEN;
        }

        static final class Mechanism {
            static final String name = "Claw: Coral";
            static final PneumaticsModuleType moduleType = REVPH;
            static final Claw.Config constants = new Claw.Config(
                    name,
                    moduleType,
                    CoralClaw.Control.solenoidOnClawValue);
        }

        static final class SolenoidConstants {
            static final int channel = 8;
        }

    }

    public static void create() {
        Pneumatics pneumatics = new REVPneumatics(Module.PneumaticsModule.constants, Module.PneumaticsModule.pneumaticHub);
        PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem(pneumatics);
        pneumaticSubsystem.register();
        SmartDashboard.putData("Turn Compressor On", pneumaticSubsystem.turnCompressorOn());
        SmartDashboard.putData("Turn Compressor Off", pneumaticSubsystem.turnCompressorOff());
        SmartDashboard.putData("Clear Sticky Faults", pneumaticSubsystem.clearFaults());
    }

    public static ClawSubsystem createAlgaeClaw() {
        Solenoid solenoid = Module.PneumaticsModule.pneumaticHub.makeSolenoid(AlgaeClaw.SolenoidConstants.channel);
        Claw claw = new SolenoidClaw(AlgaeClaw.Mechanism.constants, solenoid);
        ClawSubsystem clawSubsystem = new ClawSubsystem(claw);
        clawSubsystem.register();
        return clawSubsystem;
    }

    public static ClawSubsystem createCoralClaw() {
        Solenoid solenoid = Module.PneumaticsModule.pneumaticHub.makeSolenoid(CoralClaw.SolenoidConstants.channel);
        Claw claw = new SolenoidClaw(CoralClaw.Mechanism.constants, solenoid);
        ClawSubsystem commandCoralClaw = new ClawSubsystem(claw);
        commandCoralClaw.register();
        return commandCoralClaw;
    }
}
