package digilib.motorControllers.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import digilib.motorControllers.MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.revrobotics.spark.SparkBase.ControlType.*;

public abstract class SparkController implements MotorController {

    protected static final ClosedLoopSlot position_voltage_slot = ClosedLoopSlot.kSlot0;
    protected static final ClosedLoopSlot velocity_voltage_slot = ClosedLoopSlot.kSlot1;

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
}
