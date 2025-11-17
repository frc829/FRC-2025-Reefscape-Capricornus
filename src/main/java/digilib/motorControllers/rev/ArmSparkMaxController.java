package digilib.motorControllers.rev;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSparkMaxController extends SparkMaxController {

    private final ArmFeedforward feedforward;
    private double momentOfInertia;

    public ArmSparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            ArmFeedforward feedforward,
            double momentOfInertia,
            DCMotor dcMotor) {
        super(sparkMax, positionProfile, velocityProfile, dcMotor);
        this.feedforward = feedforward;
        this.momentOfInertia = momentOfInertia;
    }

    public ArmSparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            ArmFeedforward feedforward,
            double momentOfInertia,
            DCMotor dcMotor,
            SparkMaxSim sparkMaxSim,
            SingleJointedArmSim armSim) {
        this(sparkMax, positionProfile, velocityProfile, feedforward, momentOfInertia, dcMotor);
        new SimThread(sparkMaxSim, armSim);
    }

    @Override
    public final double getAcceleration() {
        return dcMotor.KtNMPerAmp * getCurrentAmps() / momentOfInertia;
    }

    @Override
    protected final double getFeedForward(double currentVelocitySetpoint, double nextVelocitySetpoint) {
        return feedforward.calculateWithVelocities(
                getPosition(),
                currentVelocitySetpoint,
                nextVelocitySetpoint);
    }
}
