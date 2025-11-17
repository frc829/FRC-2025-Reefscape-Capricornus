package digilib.motorControllers.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import digilib.motorControllers.MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.revrobotics.spark.SparkBase.ControlType.kPosition;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;

public class GearboxSparkController extends SparkController implements MotorController {

    private final SimpleMotorFeedforward voltageFeedforward;
    private final SimpleMotorFeedforward currentFeedforward;

    public GearboxSparkController(
            SparkBase spark,
            RelativeEncoder encoder,
            SparkClosedLoopController closedLoopController,
            ExponentialProfile positionMotionProfile,
            TrapezoidProfile velocityMotionProfile,
            PIDController positionCurrentClosedLoopController,
            PIDController velocityCurrentClosedLoopController,
            SimpleMotorFeedforward voltageFeedforward,
            SimpleMotorFeedforward currentFeedforward) {
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
    }

    @Override
    public double getAcceleration() {
        return velocityCoefficient * getVelocity() + voltageCoefficient * getVoltageVolts();
    }

    @Override
    public void applyPositionUsingVoltage(double position) {
        updatePositionStates(position);
        updatePositionProfile(voltageFeedforward.getDt());
        double feedforwardVoltage = voltageFeedforward.calculateWithVelocities(getVelocity(), positionStateCurrent.velocity);
        closedLoopController.setReference(positionStateCurrent.position, kPosition, position_voltage_slot, feedforwardVoltage, kVoltage);
    }

    @Override
    public void applyPositionUsingCurrent(double position) {
        updatePositionStates(position);
        updatePositionProfile(currentFeedforward.getDt());
        double feedforwardCurrent = currentFeedforward.calculateWithVelocities(getVelocity(), positionStateCurrent.velocity);
        double feedbackCurrent = positionCurrentClosedLoopController.calculate(getPosition(), positionStateCurrent.position);
        applyCurrent(feedforwardCurrent + feedbackCurrent);
    }

    @Override
    public void applyVelocityUsingVoltage(double velocity) {
        updateVelocityStates(velocity);
        updateVelocityProfile(voltageFeedforward.getDt());
        double feedforwardVoltage = voltageFeedforward.calculateWithVelocities(getVelocity(), velocityStateCurrent.position);
        closedLoopController.setReference(velocityStateCurrent.position, kVelocity, velocity_voltage_slot, feedforwardVoltage, kVoltage);
    }

    @Override
    public void applyVelocityUsingCurrent(double velocity) {
        updateVelocityStates(velocity);
        updateVelocityProfile(currentFeedforward.getDt());
        double feedforwardCurrent = currentFeedforward.calculateWithVelocities(getVelocity(), velocityStateCurrent.position);
        double feedbackCurrent = velocityCurrentClosedLoopController.calculate(getVelocity(), velocityStateCurrent.position);
        applyCurrent(feedforwardCurrent + feedbackCurrent);
    }
}
