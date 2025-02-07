package digilib.claws;

import static digilib.claws.ClawState.*;

public interface Claw {

    public void setControl(ClawRequest request);

    public ClawState getState();

    public ClawState getStateCopy();

    public ClawState getLastState();

    public void updateTelemetry();

    public void setValue(ClawValue state);

    public void update();



}
