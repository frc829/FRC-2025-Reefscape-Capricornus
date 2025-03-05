package digilib.claws;

import digilib.SolenoidType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static digilib.claws.ClawState.*;

public interface Claw {

    SolenoidType getSolenoidType();

    PneumaticsModuleType getPneumaticsModuleType();

    ClawState getState();

    void setValue(ClawValue state);

    void toggle();

    void update();

    void updateState();

    void updateTelemetry();
}
