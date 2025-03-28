package digilib.lidarSensor;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.wpilibj.Alert.AlertType.*;

public class LaserCanLidarSensor extends LidarSensor {
    private final LaserCanInterface laserCan;
    private final Alert alert = new Alert("Error", kError);

    public LaserCanLidarSensor(
            Config config,
            LaserCan laserCan) {
        super(config.name());
        this.laserCan = laserCan;
    }

    @Override
    public double getDistanceMillimeters() {
        if (RobotBase.isReal()) {
            Measurement measurement = laserCan.getMeasurement();
            if (measurement != null && measurement.status == LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
                alert.set(false);
                return measurement.distance_mm;
            } else if (measurement != null) {
                alert.setText("LaserCan measurement is not valid");
                alert.set(true);
                return Double.NaN;
            } else {
                alert.setText("LaserCan measurement is null");
                alert.set(true);
                return Double.NaN;
            }
        } else {
            return 0.0;
        }
    }
}
