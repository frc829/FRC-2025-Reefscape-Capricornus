package digilib.claws;

import digilib.SolenoidType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public interface Claw {

    SolenoidType getSolenoidType();

    PneumaticsModuleType getPneumaticsModuleType();

    ClawState getState();

    void setControl(ClawRequest request);

    void setValue(ClawValue state);

    void update();

    void updateState();

    void updateTelemetry();
}
