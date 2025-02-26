package frc.robot.subsystems.cameraPower;

import au.grapplerobotics.MitoCANdria;
import digilib.cameraPower.CameraPower;
import digilib.cameraPower.CameraPowerConstants;
import digilib.cameraPower.MitoCandriaPowerModule;

import static frc.robot.subsystems.cameraPower.CameraPowerSubsystemConstants.Mechanism.name;
import static frc.robot.subsystems.cameraPower.CameraPowerSubsystemConstants.Module.constants;
import static frc.robot.subsystems.cameraPower.CameraPowerSubsystemConstants.Module.module_id;


public class CameraPowerSubsystemConstants {

    static final class Mechanism{
        static final String name = "Camera Power";
    }

    public static final class Module{
        static final int module_id = 3;
        static final CameraPowerConstants constants = new CameraPowerConstants(name, module_id);
    }

    public static CameraPowerSubsystem create() {
        MitoCANdria mitoCANdria;
        try{
            mitoCANdria = new MitoCANdria(module_id);
        }catch(Exception e){
            mitoCANdria = null;
        }
        CameraPower cameraPower = new MitoCandriaPowerModule(constants, mitoCANdria);
        CameraPowerSubsystem cameraPowerSubsystem = new CameraPowerSubsystem(cameraPower);
        cameraPowerSubsystem.register();
        return cameraPowerSubsystem;
    }
}
