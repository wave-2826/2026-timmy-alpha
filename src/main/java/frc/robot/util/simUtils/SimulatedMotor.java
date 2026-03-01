package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * <h1>{@link edu.wpi.first.wpilibj.simulation.DCMotorSim} with a bit of extra spice.</h1>
 *
 * <p>This class extends the functionality of the original {@link edu.wpi.first.wpilibj.simulation.DCMotorSim} and
 * models the following aspects in addition:
 *
 * <ul>
 *   <li>Motor Controller Closed Loops.
 *   <li>Smart current limiting.
 *   <li>Friction force on the rotor.
 * </ul>
 */
public class SimulatedMotor {
    public class SimMotorState {
        public Angle mechanismAngularPosition;
        public AngularVelocity mechanismAngularVelocity;

        public SimMotorState(Angle mechanismAngularPosition, AngularVelocity mechanismAngularVelocity) {
            this.mechanismAngularPosition = mechanismAngularPosition;
            this.mechanismAngularVelocity = mechanismAngularVelocity;
        }

        public void step(Torque finalElectricTorque, Torque finalFrictionTorque, MomentOfInertia loadMOI, Time dt) {
            // Step 0: Convert all units to SI units (radians, radians per second, Newton-meters, seconds, kg*m²)
            double currentAngularPositionRadians = mechanismAngularPosition.in(Radians);
            double currentAngularVelocityRadiansPerSecond = mechanismAngularVelocity.in(RadiansPerSecond);
            final double electricTorqueNewtonsMeters = finalElectricTorque.in(NewtonMeters);
            final double frictionTorqueNewtonsMeters = finalFrictionTorque.in(NewtonMeters);
            final double loadMOIKgMetersSquared = loadMOI.in(KilogramSquareMeters);
            final double dtSeconds = dt.in(Seconds);

            // Step 1: Apply electric torque to the angular velocity.
            // The torque causes a change in the angular velocity, according to the moment of inertia.
            currentAngularVelocityRadiansPerSecond += electricTorqueNewtonsMeters / loadMOIKgMetersSquared * dtSeconds;

            // Step 2: Calculate the change in angular velocity due to friction.
            // Friction opposes the motion and reduces the angular velocity over time.
            final double deltaAngularVelocityDueToFrictionRadPerSec =
                    Math.copySign(frictionTorqueNewtonsMeters, -currentAngularVelocityRadiansPerSecond)
                            / loadMOIKgMetersSquared
                            * dtSeconds;

            // Step 3: Check if the angular velocity changes direction due to friction, or if it reaches zero.
            // If friction causes the motor to reverse direction, or if the velocity reaches zero, set the angular velocity
            // to zero.
            if ((currentAngularVelocityRadiansPerSecond + deltaAngularVelocityDueToFrictionRadPerSec)
                            * currentAngularVelocityRadiansPerSecond
                    <= 0)
                // The velocity has reversed direction or reached zero, so stop the motor
                currentAngularVelocityRadiansPerSecond = 0;
            else
                // Otherwise, apply the change due to friction
                currentAngularVelocityRadiansPerSecond += deltaAngularVelocityDueToFrictionRadPerSec;

            // Step 4: Integrate angular velocity to find the new position.
            // The new angular position is the current position plus the change in position over the time step.
            currentAngularPositionRadians += currentAngularVelocityRadiansPerSecond * dtSeconds;

            // Return a new instance with the updated angular position and velocity
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

        if (state.mechanismAngularPosition.lte(configs.reverseHardwareLimit))
            state = new SimMotorState(configs.reverseHardwareLimit, RadiansPerSecond.zero());
        else if (state.mechanismAngularPosition.gte(configs.forwardHardwareLimit))
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
     * <h2>Obtains the <strong>final</strong> position of the mechanism.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularPosition()}.
     *
     * @return the angular position of the mechanism, continuous
     */
    public Angle getAngularPosition() {
        return state.mechanismAngularPosition;
    }

    /**
     * <h2>Obtains the angular position measured by the relative encoder of the motor.</h2>
     *
     * @return the angular position measured by the encoder, continuous
     */
    public Angle getEncoderPosition() {
        return getAngularPosition().times(configs.gearing);
    }

    /**
     * <h2>Obtains the <strong>final</strong> velocity of the mechanism.</h2>
     *
     * <p>This is equivalent to {@link edu.wpi.first.wpilibj.simulation.DCMotorSim#getAngularVelocity()}.
     *
     * @return the final angular velocity of the mechanism
     */
    public AngularVelocity getVelocity() {
        return state.mechanismAngularVelocity;
    }

    /**
     * <h2>Obtains the angular velocity measured by the relative encoder of the motor.</h2>
     *
     * @return the angular velocity measured by the encoder
     */
    public AngularVelocity getEncoderVelocity() {
        return getVelocity().times(configs.gearing);
    }

    /**
     * <h2>Obtains the applied voltage by the motor controller.</h2>
     *
     * <p>The applied voltage is calculated by the motor controller in the previous call to {@link #update(Time)}
     *
     * <p>The motor controller specified by {@link #useMotorController(SimulatedMotorController)} is used to calculate
     * the applied voltage.
     *
     * <p>The applied voltage is also restricted for current limit and battery voltage.
     *
     * @return the applied voltage
     */
    public Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    /**
     * <h2>Obtains the <strong>stator</strong> current.</h2>
     *
     * <p>This is equivalent to {@link DCMotorSim#getCurrentDrawAmps()}
     *
     * @return the stator current of the motor
     */
    public Current getStatorCurrent() {
        return statorCurrent;
    }

    /**
     * <h2>Obtains the <strong>supply</strong> current.</h2>
     *
     * <p>The supply current is different from the stator current, as described <a
     * href='https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10'>here</a>.
     *
     * @return the supply current of the motor
     */
    public Current getSupplyCurrent() {
        // Supply Power = Stator Power (Conservation of Energy)
        // Hence,
        // Battery Voltage x Supply Current = Applied Voltage x Stator Current
        // Supply Current = Stator Current * Applied Voltage / Battery Voltage
        return getStatorCurrent().times(appliedVoltage.div(SimulatedBattery.getBatteryVoltage()));
    }

    /**
     * <h2>Obtains the configuration of the motor.</h2>
     *
     * <p>You can modify the configuration of this motor by:
     *
     * <pre><code>
     *     mapleMotorSim.getConfigs()
     *          .with...(...)
     *          .with...(...);
     * </code></pre>
     *
     * @return the configuration of the motor
     */
    public SimMotorConfigs getConfigs() {
        return this.configs;
    }
}