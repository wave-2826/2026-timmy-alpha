package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public final class SimMotorConfigs {
    public final DCMotor motor;
    public final double gearing;
    public final MomentOfInertia loadMOI;
    public final Torque friction;

    protected Angle forwardHardwareLimit, reverseHardwareLimit;

    public SimMotorConfigs(DCMotor motor, double gearing, MomentOfInertia loadMOI, Voltage frictionVoltage) {
        this.motor = motor;
        this.gearing = gearing;
        this.loadMOI = loadMOI;
        this.friction = NewtonMeters.of(motor.getTorque(motor.getCurrent(0, frictionVoltage.in(Volts))));

        forwardHardwareLimit = Radians.of(Double.POSITIVE_INFINITY);
        reverseHardwareLimit = Radians.of(-Double.POSITIVE_INFINITY);
    }

    public Voltage calculateVoltage(Current current, AngularVelocity mechanismVelocity) {
        return Volts.of(motor.getVoltage(current.in(Amps), mechanismVelocity.in(RadiansPerSecond) * gearing));
    }

    public AngularVelocity calculateMechanismVelocity(Current current, Voltage voltage) {
        return RadiansPerSecond.of(motor.getSpeed(motor.getTorque(current.in(Amps)), voltage.in(Volts))).div(gearing);
    }

    public Current calculateCurrent(AngularVelocity mechanismVelocity, Voltage voltage) {
        return Amps.of(motor.getCurrent(mechanismVelocity.in(RadiansPerSecond) * gearing, voltage.in(Volts)));
    }

    public Current calculateCurrent(Torque torque) {
        return Amps.of(motor.getCurrent(torque.in(NewtonMeters) / gearing));
    }

    public Torque calculateTorque(Current current) {
        return NewtonMeters.of(motor.getTorque(current.in(Amps)) * gearing);
    }

    public SimMotorConfigs withHardLimits(Angle forwardLimit, Angle reverseLimit) {
        this.forwardHardwareLimit = forwardLimit;
        this.reverseHardwareLimit = reverseLimit;
        return this;
    }

    public AngularVelocity freeSpinMechanismVelocity() {
        return RadiansPerSecond.of(motor.freeSpeedRadPerSec / gearing);
    }

    public Current freeSpinCurrent() {
        return Amps.of(motor.freeCurrentAmps);
    }

    public Current stallCurrent() {
        return Amps.of(motor.stallCurrentAmps);
    }

    public Torque stallTorque() {
        return NewtonMeters.of(motor.stallTorqueNewtonMeters);
    }

    public Voltage nominalVoltage() {
        return Volts.of(motor.nominalVoltageVolts);
    }

    @Override
    protected SimMotorConfigs clone() {
        SimMotorConfigs cfg = new SimMotorConfigs(
            motor, gearing, loadMOI, Volts.of(motor.getVoltage(friction.in(NewtonMeter), 0.0))
        ).withHardLimits(forwardHardwareLimit, reverseHardwareLimit);

        return cfg;
    }
}