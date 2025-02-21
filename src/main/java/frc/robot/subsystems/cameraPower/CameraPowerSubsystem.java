package frc.robot.subsystems.cameraPower;

import digilib.cameraPower.CameraPower;
import digilib.cameraPower.CameraPowerRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Supplier;

public class CameraPowerSubsystem implements Subsystem {
    private final CameraPower cameraPower;

    public CameraPowerSubsystem(CameraPower cameraPower) {
        this.cameraPower = cameraPower;
    }

    public Command applyRequestOnce(Supplier<CameraPowerRequest> requestSupplier) {
        return runOnce(() -> cameraPower.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        cameraPower.update();
    }
}
