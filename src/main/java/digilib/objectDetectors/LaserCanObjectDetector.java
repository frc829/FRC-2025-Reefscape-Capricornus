package digilib.objectDetectors;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LaserCanObjectDetector implements ObjectDetector {
    private final ObjectDetectorState state;
    private ObjectDetectorRequest request;
    private final ObjectDetectorTelemetry telemetry;
    private final LaserCan laserCan;
    private final Distance maxTrueDistance;
    private final Distance minTrueDistance;
    private final Alert alert = new Alert("Error", Alert.AlertType.kError);
    private final CommandXboxController simulated;

    public LaserCanObjectDetector(
            String name,
            LaserCan laserCan,
            Distance maxTrueDistance,
            Distance minTrueDistance) {
        this.laserCan = laserCan;
        this.maxTrueDistance = maxTrueDistance;
        this.minTrueDistance = minTrueDistance;
        this.state = new ObjectDetectorState();
        this.telemetry = new ObjectDetectorTelemetry(name, maxTrueDistance, minTrueDistance);
        if(RobotBase.isReal()){
            simulated = null;
        }else{
            simulated = new CommandXboxController(4);
        }
    }

    @Override
    public ObjectDetectorState getState() {
        return state;
    }

    @Override
    public void setControl(ObjectDetectorRequest request) {
        if (this.request != request) {
            this.request = request;
        }
        request.apply(this);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        if(RobotBase.isReal()){
            try {
                double meters = laserCan.getMeasurement().distance_mm / 1000.0;
                state.withDistance(meters)
                        .withInRange(meters <= maxTrueDistance.baseUnitMagnitude() && meters >= minTrueDistance.baseUnitMagnitude())
                        .withTimestamp(Timer.getFPGATimestamp());
                alert.set(false);
            } catch (Exception e) {
                alert.setText("Error: " + e.getMessage());
                alert.set(true);
            }
        }else{
            if(simulated.a().getAsBoolean()){
                state.withInRange(true);
            }else{
                state.withInRange(false);
            }
        }

    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }
}
