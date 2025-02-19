package frc.robot.subsystems.pneumatics;

import digilib.pneumatics.Pneumatics;
import digilib.pneumatics.PneumaticsConstants;
import digilib.pneumatics.REVPneumatics;
import edu.wpi.first.wpilibj.PneumaticHub;

import static frc.robot.subsystems.pneumatics.CommandPneumaticsConstants.Mechanism.*;
import static frc.robot.subsystems.pneumatics.CommandPneumaticsConstants.PneumaticsModule.constants;
import static frc.robot.subsystems.pneumatics.CommandPneumaticsConstants.PneumaticsModule.pneumaticHub;

public class CommandPneumaticsConstants {

    static final class Mechanism{
        static final String name = "Pneumatics";
    }

    public static final class PneumaticsModule{
        static final int module = 2;
        public static final PneumaticHub pneumaticHub = new PneumaticHub(module);
        static final PneumaticsConstants constants = new PneumaticsConstants(name);
    }

    public static CommandPneumatics create() {
        Pneumatics pneumatics = new REVPneumatics(constants, pneumaticHub);
        return new CommandPneumatics(pneumatics);
    }
}
