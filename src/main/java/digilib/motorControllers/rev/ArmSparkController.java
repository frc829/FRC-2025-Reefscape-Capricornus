package digilib.motorControllers.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import digilib.motorControllers.MotorController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.revrobotics.spark.SparkBase.ControlType.kPosition;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;

public class ArmSparkController extends SparkController implements MotorController {

    private final ArmFeedforward voltageFeedforward;
    private final ArmFeedforward currentFeedforward;
    private final double unmodeledDynamics;

    public ArmSparkController(
            SparkBase spark,
            RelativeEncoder encoder,
            SparkClosedLoopController closedLoopController,
            ExponentialProfile positionMotionProfile,
            TrapezoidProfile velocityMotionProfile,
            PIDController positionCurrentClosedLoopController,
            PIDController velocityCurrentClosedLoopController,
            ArmFeedforward voltageFeedforward,
            ArmFeedforward currentFeedforward) {
        super(spark,
                encoder,
                closedLoopController,
                -voltageFeedforward.getKv() / voltageFeedforward.getKa(),
                1.0 / voltageFeedforward.getKa(),
                positionMotionProfile,
                velocityMotionProfile,
                positionCurrentClosedLoopController,
                velocityCurrentClosedLoopController);
        this.voltageFeedforward = voltageFeedforward;
        this.currentFeedforward = currentFeedforward;
        this.unmodeledDynamics = -voltageFeedforward.getKg() /  voltageFeedforward.getKa();
    }

    @Override
    public double getAcceleration() {
        return velocityCoefficient * getVelocity() + voltageCoefficient * getVoltageVolts() + unmodeledDynamics * Math.cos(getPosition());
    }

    @Override
    public void applyPositionUsingVoltage(double position) {
        double currentPosition = getPosition();
        updatePositionStates(position);
        updatePositionProfile(voltageFeedforward.getDt());
        double feedforwardVoltage = voltageFeedforward.calculateWithVelocities(currentPosition, getVelocity(), positionStateCurrent.velocity);
        closedLoopController.setReference(positionStateCurrent.position, kPosition, position_voltage_slot, feedforwardVoltage, kVoltage);
    }

    @Override
    public void applyPositionUsingCurrent(double position) {
        double currentPosition = getPosition();
        updatePositionStates(position);
        updatePositionProfile(currentFeedforward.getDt());
        double feedforwardCurrent = currentFeedforward.calculateWithVelocities(currentPosition, getVelocity(), positionStateCurrent.velocity);
        double feedbackCurrent = positionCurrentClosedLoopController.calculate(currentPosition, positionStateCurrent.position);
        applyCurrent(feedforwardCurrent + feedbackCurrent);
    }

    @Override
    public void applyVelocityUsingVoltage(double velocity) {
        double currentPosition = getPosition();
        updateVelocityStates(velocity);
        updateVelocityProfile(voltageFeedforward.getDt());
        double feedforwardVoltage = voltageFeedforward.calculateWithVelocities(currentPosition, getVelocity(), velocityStateCurrent.position);
        closedLoopController.setReference(velocityStateCurrent.position, kVelocity, velocity_voltage_slot, feedforwardVoltage, kVoltage);
    }

    @Override
    public void applyVelocityUsingCurrent(double velocity) {
        double currentPosition = getPosition();
        updateVelocityStates(velocity);
        updateVelocityProfile(currentFeedforward.getDt());
        double feedforwardCurrent = currentFeedforward.calculateWithVelocities(currentPosition, getVelocity(), velocityStateCurrent.position);
        double feedbackCurrent = velocityCurrentClosedLoopController.calculate(currentPosition, velocityStateCurrent.position);
        applyCurrent(feedforwardCurrent + feedbackCurrent);
    }
}
