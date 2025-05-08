package digilib.motorControllers.rev;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimpleSparkMaxController extends SparkMaxController {

    private final SimpleMotorFeedforward feedforward;
    private double momentOfInertia;

    public SimpleSparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            SimpleMotorFeedforward feedforward,
            double momentOfInertia,
            DCMotor dcMotor) {
        super(sparkMax, positionProfile, velocityProfile, dcMotor);
        this.feedforward = feedforward;
        this.momentOfInertia = momentOfInertia;
    }

    public SimpleSparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            SimpleMotorFeedforward feedforward,
            double momentOfInertia,
            DCMotor dcMotor,
            SparkMaxSim sparkMaxSim,
            DCMotorSim dcMotorSim) {
        this(sparkMax, positionProfile, velocityProfile, feedforward, momentOfInertia, dcMotor);
        new SimThread(sparkMaxSim, dcMotorSim);
    }

    @Override
    public final double getAcceleration() {
        return dcMotor.KtNMPerAmp * getCurrentAmps() / momentOfInertia;
    }

    @Override
    protected final double getFeedForward(double currentVelocitySetpoint, double nextVelocitySetpoint) {
        return feedforward.calculateWithVelocities(currentVelocitySetpoint, nextVelocitySetpoint);
    }
}
