package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * DCMotorSim with extra features:
 * - Motor controller closed loops
 * - Smart current limiting
 * - Rotor friction force
 */
public class SimulatedMotor {
    public static final class SimMotorConfigs {
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

    public static class SimMotorState {
        public Angle mechanismAngularPosition;
        public AngularVelocity mechanismAngularVelocity;

        public SimMotorState(Angle mechanismAngularPosition, AngularVelocity mechanismAngularVelocity) {
            this.mechanismAngularPosition = mechanismAngularPosition;
            this.mechanismAngularVelocity = mechanismAngularVelocity;
        }

        public void step(Torque finalElectricTorque, Torque finalFrictionTorque, MomentOfInertia loadMOI, Time dt) {
            double currentAngularPositionRadians = mechanismAngularPosition.in(Radians);
            double currentAngularVelocityRadiansPerSecond = mechanismAngularVelocity.in(RadiansPerSecond);
            final double electricTorqueNewtonsMeters = finalElectricTorque.in(NewtonMeters);
            final double frictionTorqueNewtonsMeters = finalFrictionTorque.in(NewtonMeters);
            final double loadMOIKgMetersSquared = loadMOI.in(KilogramSquareMeters);
            final double dtSeconds = dt.in(Seconds);

            // Apply electric torque to angular velocity
            currentAngularVelocityRadiansPerSecond += electricTorqueNewtonsMeters / loadMOIKgMetersSquared * dtSeconds;

            // Friction opposes motion and reduces angular velocity
            final double deltaAngularVelocityDueToFrictionRadPerSec =
                Math.copySign(frictionTorqueNewtonsMeters, -currentAngularVelocityRadiansPerSecond)
                    / loadMOIKgMetersSquared
                    * dtSeconds;

            // If friction reverses direction or velocity reaches zero, stop the motor
            if((currentAngularVelocityRadiansPerSecond + deltaAngularVelocityDueToFrictionRadPerSec)
                * currentAngularVelocityRadiansPerSecond <= 0)
                currentAngularVelocityRadiansPerSecond = 0;
            else
                currentAngularVelocityRadiansPerSecond += deltaAngularVelocityDueToFrictionRadPerSec;

            // Integrate angular velocity to get new position
            currentAngularPositionRadians += currentAngularVelocityRadiansPerSecond * dtSeconds;

            this.mechanismAngularPosition = Radians.of(currentAngularPositionRadians);
            this.mechanismAngularVelocity = RadiansPerSecond.of(currentAngularVelocityRadiansPerSecond);
        }
    }

    private final SimMotorConfigs configs;

    private SimMotorState state;
    private SimulatedMotorController controller;
    private Voltage appliedVoltage;
    private Current statorCurrent;

    public SimulatedMotor(SimMotorConfigs configs) {
        this.configs = configs;
        this.state = new SimMotorState(Radians.zero(), RadiansPerSecond.zero());
        this.controller = (mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity) -> Volts.of(0);
        this.appliedVoltage = Volts.zero();
        this.statorCurrent = Amps.zero();

        SimulatedBattery.addMotor(this);
    }

    public void update(Time dt) {
        this.appliedVoltage = controller.updateControlSignal(
            state.mechanismAngularPosition,
            state.mechanismAngularVelocity,
            state.mechanismAngularPosition.times(configs.gearing),
            state.mechanismAngularVelocity.times(configs.gearing));
        this.appliedVoltage = SimulatedBattery.clamp(appliedVoltage);
        this.statorCurrent = configs.calculateCurrent(state.mechanismAngularVelocity, appliedVoltage);
        this.state.step(configs.calculateTorque(statorCurrent), configs.friction, configs.loadMOI, dt);

        if(state.mechanismAngularPosition.lte(configs.reverseHardwareLimit))
            state = new SimMotorState(configs.reverseHardwareLimit, RadiansPerSecond.zero());
        else if(state.mechanismAngularPosition.gte(configs.forwardHardwareLimit))
            state = new SimMotorState(configs.forwardHardwareLimit, RadiansPerSecond.zero());
    }

    public <T extends SimulatedMotorController> T useMotorController(T motorController) {
        this.controller = motorController;
        return motorController;
    }

    public SimulatedMotorController.GenericMotorController useSimpleDCMotorController() {
        return useMotorController(new SimulatedMotorController.GenericMotorController(configs.motor));
    }

    /**
     * Returns the mechanism's angular position (continuous)
     */
    public Angle getAngularPosition() {
        return state.mechanismAngularPosition;
    }

    /**
     * Returns the encoder's angular position (continuous)
     */
    public Angle getEncoderPosition() {
        return getAngularPosition().times(configs.gearing);
    }

    /**
     * Returns the mechanism's angular velocity
     */
    public AngularVelocity getVelocity() {
        return state.mechanismAngularVelocity;
    }

    /**
     * Returns the encoder's angular velocity
     */
    public AngularVelocity getEncoderVelocity() {
        return getVelocity().times(configs.gearing);
    }

    /**
     * Returns the applied voltage from the motor controller
     */
    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    /**
     * Returns the stator current
     */
    public Current getStatorCurrent() {
        return statorCurrent;
    }

    /**
     * Returns the supply current (different from stator current)
     */
    public Current getSupplyCurrent() {
        // Supply Power = Stator Power
        // Supply Current = Stator Current * Applied Voltage / Battery Voltage
        return getStatorCurrent().times(appliedVoltage.div(SimulatedBattery.getBatteryVoltage()));
    }

    public SimMotorConfigs getConfigs() {
        return this.configs;
    }
}
