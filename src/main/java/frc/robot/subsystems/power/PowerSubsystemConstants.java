package frc.robot.subsystems.power;

import digilib.power.Power;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.power.Power.*;
import static frc.robot.subsystems.power.PowerSubsystemConstants.Mechanism.name;
import static frc.robot.subsystems.power.PowerSubsystemConstants.Module.*;

public class PowerSubsystemConstants {

    static final class Mechanism{
        static final String name = "Main Power";
    }

    public static final class Module{
        static final int module = 1;
        static final ModuleType moduleType = ModuleType.kRev;
        static final Config config = new Config(name, module, moduleType);
    }

    public static void create() {
        Power power = new Power(config);
        PowerSubsystem powerSubsystem = new PowerSubsystem(power);
        powerSubsystem.register();
        SmartDashboard.putData("Clear Sticky Faults", powerSubsystem.clearFaults());
    }
}
