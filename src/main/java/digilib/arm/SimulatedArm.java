package digilib.arm;

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


public class SimulatedArm extends LinearSystemSim<N2, N1, N2> {
    private final double ks;
    private final DCMotor gearbox;
    private final double gearing;
    private final double minAngle;
    private final double maxAngle;
    private final double unmodeledAccelerationAt0Rad;

    public SimulatedArm(
            LinearSystem<N2, N1, N2> plant,
            double ks,
            DCMotor gearbox,
            double gearing,
            double unmodeledAccelerationAt0Rad,
            double startingAngleRads,
            double minAngleRads,
            double maxAngleRads,
            double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        this.ks = ks;
        this.gearbox = gearbox;
        this.gearing = gearing;
        minAngle = minAngleRads;
        maxAngle = maxAngleRads;
        this.unmodeledAccelerationAt0Rad = unmodeledAccelerationAt0Rad;

        setState(startingAngleRads, 0.0);
    }

    public final void setState(double angleRadians, double velocityRadPerSec) {
        setState(
                VecBuilder.fill(MathUtil.clamp(angleRadians, minAngle, maxAngle), velocityRadPerSec));
    }

    public boolean wouldHitLowerLimit(double currentAngleRads) {
        return currentAngleRads <= this.minAngle;
    }

    public boolean wouldHitUpperLimit(double currentAngleRads) {
        return currentAngleRads >= this.maxAngle;
    }

    public boolean hasHitLowerLimit() {
        return wouldHitLowerLimit(getAngleRads());
    }

    public boolean hasHitUpperLimit() {
        return wouldHitUpperLimit(getAngleRads());
    }

    public double getAngleRads() {
        return getOutput(0);
    }

    public double getVelocityRadPerSec() {
        return getOutput(1);
    }

    public double getCurrentDrawAmps() {
        // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
        // spinning 10x faster than the output
        var motorVelocity = m_x.get(1, 0) * gearing;
        return gearbox.getCurrent(motorVelocity, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
    }

    public void setInputVoltage(double volts) {
        volts = addFriction(volts);
        setInput(volts);
        clampInput(RobotController.getBatteryVoltage());
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        Matrix<N2, N1> updatedXhat =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
                            Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                            double alphaGrav = unmodeledAccelerationAt0Rad * Math.cos(x.get(0, 0));
                            xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                            return xdot;
                        },
                        currentXhat,
                        u,
                        dtSeconds);

        // We check for collision after updating xhat
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minAngle, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxAngle, 0);
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

    public static SimulatedArm createFromSysId(
            double ks,
            double kg,
            double kv,
            double ka,
            DCMotor gearbox,
            double gearing,
            double startingAngleRadians,
            double minHeightRadians,
            double maxHeightRadians) {
        return createFromSysId(
                ks,
                kg,
                kv,
                ka,
                gearbox,
                gearing,
                startingAngleRadians,
                minHeightRadians,
                maxHeightRadians,
                0.0,
                0.0);
    }

    public static SimulatedArm createFromSysId(
            double ks,
            double kg,
            double kv,
            double ka,
            DCMotor gearbox,
            double gearing,
            double startingAngleRadians,
            double minHeightRadians,
            double maxHeightRadians,
            double positionStdDevRadians,
            double velocityStdDevRadiansPerSecond) {
        return new SimulatedArm(
                LinearSystemId.identifyPositionSystem(kv, ka),
                ks,
                gearbox,
                gearing,
                -kg / ka,
                startingAngleRadians,
                minHeightRadians,
                maxHeightRadians,
                positionStdDevRadians,
                velocityStdDevRadiansPerSecond);
    }
}
