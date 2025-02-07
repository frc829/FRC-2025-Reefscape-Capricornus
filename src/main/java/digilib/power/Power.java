package digilib.power;

public interface Power {

    public PowerState getState();

    public PowerState getStateCopy();

    public PowerState getLastState();

    public void setControl();

    public void clearStickyFaults();

    public void update();

    public void updateSimState();

    public void updateTelemetry();

}
