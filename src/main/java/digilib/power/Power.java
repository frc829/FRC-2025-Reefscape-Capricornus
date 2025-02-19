package digilib.power;

public interface Power {

    PowerState getState();

    void setControl(PowerRequest request);

    void clearStickyFaults();

    void update();

    void updateTelemetry();

}
