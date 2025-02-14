package digilib.objectDetectors;

public interface ObjectDetector {

    ObjectDetectorState getState();

    void setControl(ObjectDetectorRequest request);

    void update();

    void updateState();

    void updateTelemetry();
}
