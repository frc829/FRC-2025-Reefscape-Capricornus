package frc.robot.mechanisms.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class DualVortexElevator extends Elevator {


    private final SparkFlex primaryMotor;
    private final SparkFlex followerMotor;
    private final SparkBaseConfig primaryMotorConfig;
    private final SparkBaseConfig followerMotorConfig;
    private final ExponentialProfile positionProfile;
    private final SlewRateLimiter velocityProfile;
    private final ClosedLoopSlot positionClosedLoopSlot;
    private final ClosedLoopSlot velocityClosedLoopSlot;
    private final ExponentialProfile.State goalState = new ExponentialProfile.State();
    private ExponentialProfile.State lastState = new ExponentialProfile.State();
    private ControlState controlState = ControlState.VELOCITY;

    public DualVortexElevator(
            ElevatorControlParameters elevatorControlParameters,
            SparkFlex primaryMotor,
            SparkFlex followerMotor,
            SparkBaseConfig primaryMotorConfig,
            SparkBaseConfig followerMotorConfig,
            ExponentialProfile positionProfile,
            SlewRateLimiter velocityProfile,
            ClosedLoopSlot positionClosedLoopSlot,
            ClosedLoopSlot velocityClosedLoopSlot) {
        super(elevatorControlParameters);
        this.primaryMotor = primaryMotor;
        this.followerMotor = followerMotor;
        this.primaryMotorConfig = primaryMotorConfig;
        this.followerMotorConfig = followerMotorConfig;
        this.positionProfile = positionProfile;
        this.velocityProfile = velocityProfile;
        this.positionClosedLoopSlot = positionClosedLoopSlot;
        this.velocityClosedLoopSlot = velocityClosedLoopSlot;

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
    public void setVelocity(LinearVelocity velocity) {

    }

    @Override
    public void setPosition(Distance position) {

    }

    @Override
    public void setHold() {

    }

    @Override
    public void setFreeFall() {

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

    private void applyVelocity(){

    }

    private void applyPosition(){

    }

    private void applyFreeFall(){
    }
}
