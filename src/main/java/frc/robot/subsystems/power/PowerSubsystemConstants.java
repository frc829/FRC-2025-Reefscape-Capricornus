package frc.robot.subsystems.power;

import digilib.power.Power;
import digilib.power.PowerConstants;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import static frc.robot.subsystems.power.PowerSubsystemConstants.Mechanism.name;
import static frc.robot.subsystems.power.PowerSubsystemConstants.Module.*;

public class PowerSubsystemConstants {

    static final class Mechanism{
        static final String name = "Main Power";
    }

    public static final class Module{
        static final int module = 1;
        static final ModuleType moduleType = ModuleType.kRev;
        static final PowerConstants constants = new PowerConstants(name, module, moduleType);
    }

    public static PowerSubsystem create() {
        Power power = new digilib.power.PowerModule(constants);
        PowerSubsystem powerSubsystem = new PowerSubsystem(power);
        powerSubsystem.register();
        return powerSubsystem;
    }
}
