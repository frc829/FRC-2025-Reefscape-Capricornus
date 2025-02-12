package digilib.objectDetectors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;

import static au.grapplerobotics.interfaces.LaserCanInterface.*;
import static edu.wpi.first.units.Units.Millimeters;

public class LaserCanObjectDetector implements ObjectDetector {

    private final LaserCan laserCan;
    private final Distance maxTrueDistance;
    private final MutDistance location = Millimeters.mutable(Double.NaN);

    public LaserCanObjectDetector(LaserCan laserCan, Distance maxTrueDistance) {
        this.laserCan = laserCan;
        this.maxTrueDistance = maxTrueDistance;
    }

    @Override
    public ObjectDetectorState getState() {
        return state;
    }

    @Override
    public void update() {
        try{
            location.mut_setMagnitude(laserCan.getMeasurement().distance_mm);
        }catch(Exception e){
            System.out.println(e.getMessage());
        }
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
