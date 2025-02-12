package digilib.objectDetectors;

public interface ObjectDetector {

    ObjectDetectorState getState();

    void update();
}
