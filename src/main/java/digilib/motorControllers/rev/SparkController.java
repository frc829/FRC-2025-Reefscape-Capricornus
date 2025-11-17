package digilib.motorControllers.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import digilib.motorControllers.MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.revrobotics.spark.SparkBase.ControlType.*;

public abstract class SparkController implements MotorController {

    public enum SparkModel {
        SparkFlex,
        SparkMax
    }

    public static final ClosedLoopSlot position_voltage_slot = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot velocity_voltage_slot = ClosedLoopSlot.kSlot1;

    private final SparkBase spark;
    private final RelativeEncoder encoder;
    protected final SparkClosedLoopController closedLoopController;
    protected final double velocityCoefficient;
    protected final double voltageCoefficient;

    private final ExponentialProfile positionMotionProfile;
    private final TrapezoidProfile velocityMotionProfile;

    protected ExponentialProfile.State positionStateCurrent = new ExponentialProfile.State();
    private final ExponentialProfile.State positionStateGoal = new ExponentialProfile.State();
    protected TrapezoidProfile.State velocityStateCurrent = new TrapezoidProfile.State();
    private final TrapezoidProfile.State velocityStateGoal = new TrapezoidProfile.State();

    protected final PIDController positionCurrentClosedLoopController;
    protected final PIDController velocityCurrentClosedLoopController;

    protected SparkController(SparkBase spark,
                              RelativeEncoder encoder,
                              SparkClosedLoopController closedLoopController,
                              double velocityCoefficient,
                              double voltageCoefficient,
                              ExponentialProfile positionMotionProfile,
                              TrapezoidProfile velocityMotionProfile,
                              PIDController positionCurrentClosedLoopController,
                              PIDController velocityCurrentClosedLoopController) {
        this.spark = spark;
        this.encoder = encoder;
        this.closedLoopController = closedLoopController;
        this.velocityCoefficient = velocityCoefficient;
        this.voltageCoefficient = voltageCoefficient;
        this.positionMotionProfile = positionMotionProfile;
        this.velocityMotionProfile = velocityMotionProfile;
        this.positionCurrentClosedLoopController = positionCurrentClosedLoopController;
        this.velocityCurrentClosedLoopController = velocityCurrentClosedLoopController;
    }

    @Override
    public double getVoltageVolts() {
        return spark.getBusVoltage() * spark.getAppliedOutput();
    }

    @Override
    public double getCurrentAmps() {
        return spark.getAppliedOutput();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void applyVoltage(double voltage) {
        spark.setVoltage(voltage);
    }

    @Override
    public void applyCurrent(double currentAmps) {
        closedLoopController.setReference(currentAmps, kCurrent);
    }

    protected void updatePositionProfile(double dt) {
        positionStateCurrent = positionMotionProfile.calculate(dt, positionStateCurrent, positionStateGoal);
    }

    protected void updateVelocityProfile(double dt) {
        velocityStateCurrent = velocityMotionProfile.calculate(dt, velocityStateCurrent, velocityStateGoal);
    }

    protected void updatePositionStates(double goalPosition) {
        positionStateGoal.position = goalPosition;
        velocityStateCurrent.position = getVelocity();
        velocityStateCurrent.velocity = getAcceleration();
    }

    protected void updateVelocityStates(double goalVelocity) {
        velocityStateGoal.position = goalVelocity;
        positionStateCurrent.position = getPosition();
        positionStateCurrent.velocity = getVelocity();
    }

    public static SparkBase create(
            int deviceId,
            MotorType motorType,
            SparkModel sparkModel,
            int depth,
            int periodMs,
            double positionConversionFactor,
            double velocityConversionFactor,
            IdleMode idleMode,
            boolean inverted,
            int smartCurrentLimit,
            FeedbackSensor feedbackSensor,
            double voltagePositionKp,
            double voltagePositionKd,
            double voltageVelocityKp) {

        SparkBase spark = sparkModel == SparkModel.SparkFlex
                ? new SparkFlex(deviceId, motorType)
                : new SparkMax(deviceId, motorType);

        EncoderConfig encoderConfig = switch (sparkModel) {
            case SparkFlex -> new EncoderConfig()
                    .quadratureAverageDepth(depth)
                    .quadratureMeasurementPeriod(periodMs);
            default -> new EncoderConfig()
                    .uvwAverageDepth(depth)
                    .uvwMeasurementPeriod(periodMs);
        };

        encoderConfig.positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        SparkBaseConfig sparkBaseConfig = switch (sparkModel) {
            case SparkFlex -> new SparkFlexConfig();
            default -> new SparkMaxConfig();
        };

        ClosedLoopConfig closedLoopConfig = new  ClosedLoopConfig()
                .feedbackSensor(feedbackSensor)
                .p(voltagePositionKp, position_voltage_slot)
                .d(voltagePositionKd, position_voltage_slot)
                .p(voltageVelocityKp, velocity_voltage_slot);

        sparkBaseConfig
                .apply(encoderConfig)
                .apply(closedLoopConfig)
                .inverted(inverted)
                .idleMode(idleMode)
                .smartCurrentLimit(smartCurrentLimit);

        return spark;
    }
}
