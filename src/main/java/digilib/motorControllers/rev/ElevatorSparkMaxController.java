package digilib.motorControllers.rev;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSparkMaxController extends SparkMaxController {

    private final ElevatorFeedforward feedforward;
    private final double massKilograms;
    private final double drumRadiusMeters;

    public ElevatorSparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            ElevatorFeedforward feedforward,
            double massKilograms,
            double drumRadiusMeters,
            DCMotor dcMotor) {
        super(sparkMax, positionProfile, velocityProfile, dcMotor);
        this.feedforward = feedforward;
        this.massKilograms = massKilograms;
        this.drumRadiusMeters = drumRadiusMeters;
    }

    public ElevatorSparkMaxController(
            SparkMax sparkMax,
            ExponentialProfile positionProfile,
            TrapezoidProfile velocityProfile,
            ElevatorFeedforward feedforward,
            double massKilograms,
            double drumRadiusMeters,
            DCMotor dcMotor,
            SparkMaxSim sparkMaxSim,
            ElevatorSim elevatorSim) {
        this(sparkMax, positionProfile, velocityProfile, feedforward, massKilograms, drumRadiusMeters, dcMotor);
        new SimThread(sparkMaxSim, elevatorSim);
    }

    @Override
    public final double getAcceleration() {
        return dcMotor.KtNMPerAmp * getCurrentAmps() / drumRadiusMeters / massKilograms;
    }

    @Override
    protected final double getFeedForward(double currentVelocitySetpoint, double nextVelocitySetpoint) {
        return feedforward.calculateWithVelocities(currentVelocitySetpoint, nextVelocitySetpoint);
    }
}
