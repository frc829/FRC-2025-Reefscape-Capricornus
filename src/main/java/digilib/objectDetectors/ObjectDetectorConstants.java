package digilib.objectDetectors;

import edu.wpi.first.units.measure.Distance;

public record ObjectDetectorConstants(String name,
                                      Distance maxTrueDistance,
                                      Distance minTrueDistance) {
}
