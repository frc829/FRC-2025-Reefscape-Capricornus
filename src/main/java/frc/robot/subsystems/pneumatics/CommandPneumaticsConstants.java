package frc.robot.subsystems.pneumatics;

import digilib.pneumatics.Pneumatics;
import digilib.pneumatics.REVPneumatics;
import edu.wpi.first.wpilibj.PneumaticHub;

import static frc.robot.subsystems.pneumatics.CommandPneumaticsConstants.PneumaticsModule.pneumaticHub;

public class CommandPneumaticsConstants {

    public static final class PneumaticsModule{
        static final int module = 2;
        public static final PneumaticHub pneumaticHub = new PneumaticHub(module);
    }

    public static CommandPneumatics create() {
        Pneumatics pneumatics = new REVPneumatics("Pneumatics", pneumaticHub);
        return new CommandPneumatics(pneumatics);
    }
}
