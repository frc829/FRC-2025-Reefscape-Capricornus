package frc.robot.mechanisms.arm;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class SimArm {

    private final DCMotor dcMotor;
    private final double reduction;
    private final Voltage ks;
    private final Distance armLength;
    private final Angle minPosition;
    private final Angle maxPosition;
    private final AngularAcceleration accelerationFromGravity;
    private final MomentOfInertia momentOfInertia;
    private final LinearSystem<N2, N1, N2> linearSystem;
    private final MutAngle angle = Radians.mutable(0.0);
    private final MutAngularVelocity angularVelocity = RadiansPerSecond.mutable(0.0);
    private final MutAngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.mutable(0.0);
    private final MutAngle rotorAngle = Radians.mutable(0.0);
    private final MutAngularVelocity rotorVelocity = RadiansPerSecond.mutable(0.0);
    private final MutAngularAcceleration rotorAcceleration = RadiansPerSecondPerSecond.mutable(0.0);
    private final MutVoltage motorVoltage = Volts.mutable(0.0);
    private final MutCurrent motorCurrent = Amps.mutable(0.0);
    private final MutTorque motorTorque = NewtonMeters.mutable(0.0);
    protected final Matrix<N2, N1> x = MatBuilder.fill(Nat.N2(), Nat.N1(), 0, 0);
    protected final Matrix<N1, N1> u = MatBuilder.fill(Nat.N1(), Nat.N1(), 0, 0);
    protected final Matrix<N2, N1> y = MatBuilder.fill(Nat.N2(), Nat.N1(), 0, 0);
    private final Matrix<N2, N1> measurementStdDevs;
    private final Matrix<N2, N1> xdot = MatBuilder.fill(Nat.N2(), Nat.N1(), 0, 0);

    public SimArm(
            DCMotor dcMotor,
            double reduction,
            Voltage ks,
            Voltage kg,
            Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
            Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
            Distance armLength,
            Angle minPosition,
            Angle maxPosition,
            Angle startingPosition,
            Angle positionStdDev,
            AngularVelocity velocityStdDev) {
        this.dcMotor = dcMotor;
        this.reduction = reduction;
        this.ks = ks;
        this.armLength = armLength;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        accelerationFromGravity = (AngularAcceleration) kg.unaryMinus().div(ka);
        linearSystem = LinearSystemId.identifyPositionSystem(
                kv.baseUnitMagnitude(),
                ka.baseUnitMagnitude());
        momentOfInertia = KilogramSquareMeters.of(reduction * dcMotor.KtNMPerAmp * ka.baseUnitMagnitude() / dcMotor.rOhms);
        measurementStdDevs = MatBuilder.fill(Nat.N2(), Nat.N1(), positionStdDev.baseUnitMagnitude(), velocityStdDev.baseUnitMagnitude());
        setState(startingPosition, RadiansPerSecond.of(0.0));
    }

    public void setState(Angle angle, AngularVelocity angularVelocity) {
        x.set(0, 0, angle.baseUnitMagnitude());
        x.set(1, 0, angularVelocity.baseUnitMagnitude());
    }

    public Angle getAngle() {
        return angle;
    }

    public AngularVelocity getAngularVelocity() {
        return angularVelocity;
    }

    public Angle getRotorAngle() {
        return rotorAngle;
    }

    public AngularVelocity getRotorVelocity() {
        return rotorVelocity;
    }

    public Voltage getMotorVoltage() {
        return motorVoltage;
    }

    public Current getMotorCurrent() {
        return motorCurrent;
    }


    public boolean wouldHitLowerLimit() {
        return angle.lte(minPosition);
    }

    public boolean wouldHitUpperLimit() {
        return angle.gte(maxPosition);
    }

    public void update(Time dt, Voltage inputVoltage) {
        u.set(0, 0, inputVoltage.baseUnitMagnitude());
        motorVoltage.mut_setMagnitude(u.get(0, 0));
        addFriction();
        updateX(dt);
        y.assignBlock(0, 0, linearSystem.calculateY(x, u));
        y.assignBlock(0, 0, y.plus(StateSpaceUtil.makeWhiteNoiseVector(measurementStdDevs)));
        angle.mut_setMagnitude(y.get(0, 0));
        angularVelocity.mut_setMagnitude(y.get(1, 0));
        rotorAngle.mut_setMagnitude(y.get(0, 0) * reduction);
        rotorVelocity.mut_setMagnitude(y.get(1, 0) * reduction);
        motorCurrent.mut_setMagnitude(dcMotor.getCurrent(
                rotorVelocity.in(RadiansPerSecond),
                motorVoltage.baseUnitMagnitude()));
        motorTorque.mut_setMagnitude(dcMotor.getTorque(motorCurrent.baseUnitMagnitude()));
        rotorAcceleration.mut_setMagnitude(motorTorque.baseUnitMagnitude() / momentOfInertia.baseUnitMagnitude());
        angularAcceleration.mut_setMagnitude(rotorAcceleration.baseUnitMagnitude() / reduction);
    }

    private void addFriction() {
        if (Math.abs(u.get(0, 0)) < ks.baseUnitMagnitude()) {
            u.set(0, 0, 0);
        } else if (u.get(0, 0) > 0.0) {
            u.set(0, 0, u.get(0, 0) - ks.baseUnitMagnitude());
        } else {
            u.set(0, 0, u.get(0, 0) + ks.baseUnitMagnitude());
        }
    }


    private void updateX(Time dt) {
        // The torque on the arm is given by τ = F⋅r, where F is the force applied by
        // gravity and r the distance from pivot to center of mass. Recall from
        // dynamics that the sum of torques for a rigid body is τ = J⋅α, were τ is
        // torque on the arm, J is the mass-moment of inertia about the pivot axis,
        // and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
        //
        // We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
        //
        //   α = (m⋅g⋅cos(θ))⋅r/J
        //
        // Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
        // arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
        // rod rotating about it's end, where L is the overall rod length. The mass
        // distribution is assumed to be uniform. Substitute r=L/2 to find:
        //
        //   α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
        //   α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
        //   α = 3/2⋅g⋅cos(θ)/L
        //
        // This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
        //
        //   f(x, u) = Ax + Bu + [0  α]ᵀ
        //   f(x, u) = Ax + Bu + [0  3/2⋅g⋅cos(θ)/L]ᵀ


        Matrix<N2, N1> updatedXhat =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
                            xdot.assignBlock(0, 0, linearSystem.getA().times(x).plus(linearSystem.getB().times(_u)));
                            double alphaGrav = 3.0 / 2.0
                                    * accelerationFromGravity.baseUnitMagnitude()
                                    * Math.cos(x.get(0, 0))
                                    / armLength.baseUnitMagnitude();
                            xdot.assignBlock(0, 0, xdot.plus(VecBuilder.fill(0, alphaGrav)));
                            return xdot;
                        },
                        x,
                        u,
                        dt.baseUnitMagnitude());

        // We check for collision after updating xhat
        if (wouldHitLowerLimit()) {
            updatedXhat.set(0, 0, minPosition.baseUnitMagnitude());
            updatedXhat.set(1, 0, 0.0);
        }
        if (wouldHitUpperLimit()) {
            updatedXhat.set(0, 0, maxPosition.baseUnitMagnitude());
            updatedXhat.set(1, 0, 0.0);
        }
        x.assignBlock(0, 0, updatedXhat);
    }

    public MomentOfInertia getMomentOfInertia() {
        return momentOfInertia;
    }

    public MutTorque getMotorTorque() {
        return motorTorque;
    }

    public MutAngularAcceleration getAngularAcceleration() {
        return angularAcceleration;
    }

    public MutAngularAcceleration getRotorAcceleration() {
        return rotorAcceleration;
    }
}

