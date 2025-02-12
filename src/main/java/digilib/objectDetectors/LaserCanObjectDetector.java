package digilib.objectDetectors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Timer;

import static au.grapplerobotics.interfaces.LaserCanInterface.*;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;

public class LaserCanObjectDetector implements ObjectDetector {
    private final ObjectDetectorState state;
    private final ObjectDetectorTelemetry telemetry;
    private final LaserCan laserCan;
    private final Distance maxTrueDistance;
    private final Distance minTrueDistance;
    private final MutDistance location = Millimeters.mutable(Double.NaN);
    private final MutTime timestamp = Seconds.mutable(0.0);

    public LaserCanObjectDetector(
            String name,
            LaserCan laserCan,
            Distance maxTrueDistance,
            Distance minTrueDistance) {
        this.laserCan = laserCan;
        this.maxTrueDistance = maxTrueDistance;
        this.minTrueDistance = minTrueDistance;
        this.state = new ObjectDetectorState();
        this.telemetry = new ObjectDetectorTelemetry(name, maxTrueDistance,minTrueDistance);
    }

    @Override
    public ObjectDetectorState getState() {
        return state;
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        try{
            location.mut_setMagnitude(laserCan.getMeasurement().distance_mm);
            state.withDistance(location.magnitude());
            state.withInRange(location.gte(minTrueDistance) && location.lte(maxTrueDistance));
            state.withTimestamp(timestamp.mut_setMagnitude(Timer.getFPGATimestamp()));
        }catch(Exception e){
            System.out.println(e.getMessage());
        }
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    public static LaserCan createLaserCan(int can_id, RangingMode rangingMode) {
        LaserCan laserCan;
        try {
            laserCan = new LaserCan(can_id);
        } catch (Exception e) {
            System.out.println("Error creating laser can");
            return null;
        }

        try {
            laserCan.setRangingMode(rangingMode);
            return laserCan;
        } catch (ConfigurationFailedException e) {
            System.out.println("Error setting laser can ranging mode");
            return null;
        } catch (NullPointerException e) {
            System.out.println("Laser Can does not exist");
            return null;
        } catch (Exception e) {
            System.out.println(e.getMessage());
            return null;
        }
    }
}
