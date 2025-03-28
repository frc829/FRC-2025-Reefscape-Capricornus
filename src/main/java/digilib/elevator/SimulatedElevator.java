package digilib.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class SimulatedElevator extends LinearSystemSim<N2, N1, N2> {
    private final double ks;
    private final DCMotor gearbox;

    private final double minHeight;

    private final double maxHeight;

    private final double unmodeledAcceleration;

    public SimulatedElevator(
            LinearSystem<N2, N1, N2> plant,
            double ks,
            DCMotor gearbox,
            double unmodeledAcceleration,
            double startingHeightMeters,
            double minHeightMeters,
            double maxHeightMeters,
            double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        this.ks = ks;
        this.gearbox = gearbox;
        minHeight = minHeightMeters;
        maxHeight = maxHeightMeters;
        this.unmodeledAcceleration = unmodeledAcceleration;

        setState(startingHeightMeters, 0);
    }

    public final void setState(double positionMeters, double velocityMetersPerSecond) {
        setState(
                VecBuilder.fill(
                        MathUtil.clamp(positionMeters, minHeight, maxHeight), velocityMetersPerSecond));
    }

    public boolean wouldHitLowerLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters <= this.minHeight;
    }

    public boolean wouldHitUpperLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters >= this.maxHeight;
    }

    public double getVelocityMetersPerSecond() {
        return getOutput(1);
    }

    @SuppressWarnings("unused")
    public double getCurrentDrawAmps() {
        // I = V / R - omega / (Kv * R)
        // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
        // spinning 10x faster than the output
        // v = r w, so w = v/r
        double kA = 1 / m_plant.getB().get(1, 0);
        double kV = -m_plant.getA().get(1, 1) * kA;
        double motorVelocityRadPerSec = m_x.get(1, 0) * kV * gearbox.KvRadPerSecPerVolt;
        var appliedVoltage = m_u.get(0, 0);
        return gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
                * Math.signum(appliedVoltage);
    }

    public void setInputVoltage(double volts) {
        volts = addFriction(volts);
        setInput(volts);
        clampInput(RobotController.getBatteryVoltage());
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat =
                NumericalIntegration.rkdp(
                        (x, _u) -> {
                            Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                            xdot = xdot.plus(VecBuilder.fill(0, unmodeledAcceleration));
                            return xdot;
                        },
                        currentXhat,
                        u,
                        dtSeconds);

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxHeight, 0);
        }
        return updatedXhat;
    }

    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage) {
        if (Math.abs(motorVoltage) < ks) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= ks;
        } else {
            motorVoltage += ks;
        }
        return motorVoltage;
    }

    public static SimulatedElevator createFromSysId(
            double ks,
            double kg,
            double kv,
            double ka,
            DCMotor gearbox,
            double startingHeightMeters,
            double minHeightMeters,
            double maxHeightMeters) {
        return createFromSysId(
                ks,
                kg,
                kv,
                ka,
                gearbox,
                startingHeightMeters,
                minHeightMeters,
                maxHeightMeters,
                0.0,
                0.0);
    }

    public static SimulatedElevator createFromSysId(
            double ks,
            double kg,
            double kv,
            double ka,
            DCMotor gearbox,
            double startingHeightMeters,
            double minHeightMeters,
            double maxHeightMeters,
            double positionStdDevRadians,
            double velocityStdDevRadiansPerSecond) {
        return new SimulatedElevator(
                LinearSystemId.identifyPositionSystem(kv, ka),
                ks,
                gearbox,
                -kg / ka,
                startingHeightMeters,
                minHeightMeters,
                maxHeightMeters,
                positionStdDevRadians,
                velocityStdDevRadiansPerSecond);
    }


}
