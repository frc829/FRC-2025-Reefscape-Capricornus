package frc.robot.mechanisms.elevator;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DualVortexElevator extends Elevator{

    private final SparkFlex primaryMotor;
    private final SparkFlex followerMotor;
    private final SparkBaseConfig primaryMotorConfig;
    private final SparkBaseConfig followerMotorConfig;

    public DualVortexElevator(
            ElevatorControlParameters elevatorControlParameters,
            SparkFlex primaryMotor,
            SparkFlex followerMotor,
            SparkBaseConfig primaryMotorConfig,
            SparkBaseConfig followerMotorConfig) {
        super(elevatorControlParameters);
        this.primaryMotor = primaryMotor;
        this.followerMotor = followerMotor;
        this.primaryMotorConfig = primaryMotorConfig;
        this.followerMotorConfig = followerMotorConfig;
    }

    @Override
    public boolean setNeutralModeToBrake() {
        return false;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        return false;
    }

    @Override
    public boolean setVelocity(LinearVelocity velocity) {
        return false;
    }

    @Override
    public boolean setPosition(Distance position) {
        return false;
    }

    @Override
    public boolean setHold() {
        return false;
    }

    @Override
    public boolean allowFall() {
        return false;
    }

    @Override
    public void resetPosition() {

    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void updateTelemetry() {

    }
}
