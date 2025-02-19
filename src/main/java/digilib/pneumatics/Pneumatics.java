package digilib.pneumatics;

public interface Pneumatics {

    PneumaticsState getState();

    void setControl(PneumaticsRequest request);

    void clearStickyFaults();

    void turnOnCompressor();

    void turnOffCompressor();

    void update();

    void updateState();

    void updateTelemetry();
}
