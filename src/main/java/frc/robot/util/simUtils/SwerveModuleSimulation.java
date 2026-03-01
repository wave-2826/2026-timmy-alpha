package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

public class SwerveModuleSimulation {
    public static class SwerveModuleSimulationConfig implements Supplier<SwerveModuleSimulation> {
        public final SimMotorConfigs driveMotorConfigs, steerMotorConfigs;
        public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEELS_COEFFICIENT_OF_FRICTION;
        public final Voltage DRIVE_FRICTION_VOLTAGE;
        public final Distance WHEEL_RADIUS;

        /**
         * <h2>Constructs a Configuration for Swerve Module Simulation.</h2>
         *
         * <p>If you are using {@link SimulatedArena#overrideSimulationTimings(Time, int)} to use custom timings, you must
         * call the method before constructing any swerve module simulations using this constructor.
         *
         * @param driveMotorModel the model of the driving motor
         * @param steerMotorModel; the model of the steering motor
         * @param driveGearRatio the gear ratio for the driving motor, >1 is reduction
         * @param steerGearRatio the gear ratio for the steering motor, >1 is reduction
         * @param driveFrictionVoltage the measured minimum amount of voltage that can turn the driving rotter
         * @param steerFrictionVoltage the measured minimum amount of voltage that can turn the steering rotter
         * @param wheelRadius the radius of the wheels.
         * @param steerRotationalInertia the rotational inertia of the entire steering mechanism
         * @param wheelsCoefficientOfFriction the <a
         *     href='https://simple.wikipedia.org/wiki/Coefficient_of_friction#:~:text=A%20coefficient%20of%20friction%20is%20a%20value%20that%20shows%20the'>coefficient
         *     of friction</a> of the tires, normally around 1.2 {@link Units#inchesToMeters(double)}.
         */
        public SwerveModuleSimulationConfig(
                DCMotor driveMotorModel,
                DCMotor steerMotorModel,
                double driveGearRatio,
                double steerGearRatio,
                Voltage driveFrictionVoltage,
                Voltage steerFrictionVoltage,
                Distance wheelRadius,
                MomentOfInertia steerRotationalInertia,
                double wheelsCoefficientOfFriction) {
            this.driveMotorConfigs =
                    new SimMotorConfigs(driveMotorModel, driveGearRatio, KilogramSquareMeters.zero(), driveFrictionVoltage);
            this.steerMotorConfigs =
                    new SimMotorConfigs(steerMotorModel, steerGearRatio, steerRotationalInertia, steerFrictionVoltage);
            DRIVE_GEAR_RATIO = driveGearRatio;
            STEER_GEAR_RATIO = steerGearRatio;
            WHEELS_COEFFICIENT_OF_FRICTION = wheelsCoefficientOfFriction;
            DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;
            WHEEL_RADIUS = wheelRadius;
        }

        @Override
        public SwerveModuleSimulation get() {
            return new SwerveModuleSimulation(this);
        }

        public double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {
            return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;
        }

        /**
         *
         *
         * <h2>Obtains the theoretical speed that the module can achieve.</h2>
         *
         * @return the theoretical maximum ground speed that the module can achieve, in m/s
         */
        public LinearVelocity maximumGroundSpeed() {
            return MetersPerSecond.of(
                    driveMotorConfigs.freeSpinMechanismVelocity().in(RadiansPerSecond) * WHEEL_RADIUS.in(Meters));
        }

        /**
         *
         *
         * <h2>Obtains the theoretical maximum propelling force of ONE module.</h2>
         *
         * <p>Calculates the maximum propelling force with respect to the gripping force and the drive motor's torque under
         * its current limit.
         *
         * @param robotMass the mass of the robot
         * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
         * @return the maximum propelling force of EACH module
         */
        public Force getTheoreticalPropellingForcePerModule(Mass robotMass, int modulesCount, Current statorCurrentLimit) {
            final double
                    maxThrustNewtons =
                            driveMotorConfigs.calculateTorque(statorCurrentLimit).in(NewtonMeters)
                                    / WHEEL_RADIUS.in(Meters),
                    maxGrippingNewtons = 9.8 * robotMass.in(Kilograms) / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;

            return Newtons.of(Math.min(maxThrustNewtons, maxGrippingNewtons));
        }

        /**
         *
         *
         * <h2>Obtains the theatrical linear acceleration that the robot can achieve.</h2>
         *
         * <p>Calculates the maximum linear acceleration of a robot, with respect to its mass and
         * {@link #getTheoreticalPropellingForcePerModule(Mass, int, Current)}.
         *
         * @param robotMass the mass of the robot
         * @param modulesCount the amount of modules on the robot, assumed to be sharing the gravity force equally
         */
        public LinearAcceleration maxAcceleration(Mass robotMass, int modulesCount, Current statorCurrentLimit) {
            return getTheoreticalPropellingForcePerModule(robotMass, modulesCount, statorCurrentLimit)
                    .times(modulesCount)
                    .div(robotMass);
        }
    }

    public final SwerveModuleSimulationConfig config;

    public final SimulatedMotor steerMotorSim;

    private Voltage driveMotorAppliedVoltage = Volts.zero();
    private Current driveMotorStatorCurrent = Amps.zero();
    private Angle driveWheelFinalPosition = Radians.zero();
    private AngularVelocity driveWheelFinalSpeed = RadiansPerSecond.zero();

    private SimulatedMotorController driveMotorController;

    private final Angle steerRelativeEncoderOffSet = Radians.of((Math.random() - 0.5) * 30);
    private final Queue<Angle> driveWheelFinalPositionCache;
    private final Queue<Rotation2d> steerAbsolutePositionCache;

    /**
     * <h2>Constructs a Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link SimulatedArena#overrideSimulationTimings(Time, int)} to use custom timings, you must
     * call the method before constructing any swerve module simulations using this constructor.
     *
     * @param config the configuration
     */
    public SwerveModuleSimulation(SwerveModuleSimulationConfig config) {
        this.config = config;

        SimulatedBattery.addElectricalAppliances(this::getDriveMotorSupplyCurrent);
        this.steerMotorSim = new SimulatedMotor(config.steerMotorConfigs);

        this.driveWheelFinalPositionCache = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < Simulation.subTicks; i++)
            driveWheelFinalPositionCache.offer(driveWheelFinalPosition);
        this.steerAbsolutePositionCache = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < Simulation.subTicks; i++)
            steerAbsolutePositionCache.offer(getSteerAbsoluteFacing());

        this.driveMotorController = new SimulatedMotorController.GenericMotorController(config.driveMotorConfigs.motor);
        this.steerMotorSim.useSimpleDCMotorController();
    }

    public SimMotorConfigs getDriveMotorConfigs() {
        return config.driveMotorConfigs;
    }

    public SimMotorConfigs getSteerMotorConfigs() {
        return steerMotorSim.getConfigs();
    }

    /**
     * <h2>Sets the motor controller for the drive motor.</h2>
     *
     * <p>The configured controller runs control loop on the motor.
     *
     * @param driveMotorController the motor controller to control the drive motor
     */
    public <T extends SimulatedMotorController> T useDriveMotorController(T driveMotorController) {
        this.driveMotorController = driveMotorController;
        return driveMotorController;
    }

    public SimulatedMotorController.GenericMotorController useGenericMotorControllerForDrive() {
        return useDriveMotorController(
                new SimulatedMotorController.GenericMotorController(config.driveMotorConfigs.motor));
    }

    /**
     * <h2>Requests the Steering Motor to Run at a Specified Output.</h2>
     *
     * <p>Think of it as the <code>requestOutput()</code> of your physical steering motor.
     *
     * @param steerMotorController the motor controller to control the steer motor
     */
    public <T extends SimulatedMotorController> T useSteerMotorController(T steerMotorController) {
        return this.steerMotorSim.useMotorController(steerMotorController);
    }

    public SimulatedMotorController.GenericMotorController useGenericControllerForSteer() {
        return this.steerMotorSim.useSimpleDCMotorController();
    }

    /**
     * <h2>Updates the Simulation for This Module.</h2>
     *
     * <p><strong>Note:</strong> Friction forces are not simulated in this method.
     *
     * @param moduleCurrentGroundVelocityWorldRelative the current ground velocity of the module, relative to the world
     * @param robotFacing the absolute facing of the robot, relative to the world
     * @param gravityForceOnModuleNewtons the gravitational force acting on this module, in newtons
     * @return the propelling force generated by the module, as a {@link Vector2} object
     */
    public Translation2d updateSimulationSubTickGetModuleForce(
        Translation2d moduleCurrentGroundVelocityWorldRelative,
        Rotation2d robotFacing,
        double gravityForceOnModuleNewtons) {
        /* Step1: Update the steer mechanism simulation */
        steerMotorSim.update(Simulation.simulationDt);

        /* Step2: Simulate the amount of propelling force generated by the module. */
        final double grippingForceNewtons = config.getGrippingForceNewtons(gravityForceOnModuleNewtons);
        final Rotation2d moduleWorldFacing = this.getSteerAbsoluteFacing().plus(robotFacing);
        final Translation2d propellingForce = getPropellingForce(grippingForceNewtons, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);

        /* Step3: Updates and caches the encoder readings for odometry simulation. */
        updateEncoderCaches();

        return propellingForce;
    }

    /**
     * <h2>Calculates the amount of propelling force that the module generates.</h2>
     *
     * <p>For most of the time, that propelling force is directly applied to the drivetrain. And the drive wheel runs as
     * fast as the ground velocity
     *
     * <p>However, if the propelling force exceeds the gripping, only the max gripping force is applied. The rest of the
     * propelling force will cause the wheel to start skidding and make the odometry inaccurate.
     *
     * @param grippingForceNewtons the amount of gripping force that wheel can generate, in newtons
     * @param moduleWorldFacing the current world facing of the module
     * @param moduleCurrentGroundVelocity the current ground velocity of the module, world-reference
     * @return a vector representing the propelling force that the module generates, world-reference
     */
    private Translation2d getPropellingForce(double grippingForceNewtons, Rotation2d moduleWorldFacing, Translation2d moduleCurrentGroundVelocity) {
        final double driveWheelTorque = getDriveWheelTorque();
        double propellingForceNewtons = driveWheelTorque / config.WHEEL_RADIUS.in(Meters);
        final boolean skidding = Math.abs(propellingForceNewtons) > grippingForceNewtons;
        if(skidding) propellingForceNewtons = Math.copySign(grippingForceNewtons, propellingForceNewtons);

        final double floorVelocityProjectionOnWheelDirectionMPS = moduleCurrentGroundVelocity.getNorm()
            * moduleCurrentGroundVelocity.getAngle().minus(moduleWorldFacing).getCos();

        // if the chassis is tightly gripped on floor, the floor velocity is projected to the wheel
        this.driveWheelFinalSpeed =
            RadiansPerSecond.of(floorVelocityProjectionOnWheelDirectionMPS / config.WHEEL_RADIUS.in(Meters));

        // if the module is skidding
        if(skidding) {
            final AngularVelocity skiddingEquilibriumWheelSpeed = config.driveMotorConfigs.calculateMechanismVelocity(
                config.driveMotorConfigs.calculateCurrent(
                    NewtonMeters.of(propellingForceNewtons * config.WHEEL_RADIUS.in(Meters))),
                driveMotorAppliedVoltage);
            this.driveWheelFinalSpeed = driveWheelFinalSpeed.times(0.5).plus(skiddingEquilibriumWheelSpeed.times(0.5));
        }

        return new Translation2d(propellingForceNewtons, moduleWorldFacing);
    }

    /**
     * <h2>Calculates the amount of torque that the drive motor can generate on the wheel.</h2>
     *
     * @return the amount of torque on the wheel by the drive motor, in Newton * Meters
     */
    private double getDriveWheelTorque() {
        driveMotorAppliedVoltage = driveMotorController.updateControlSignal(
                driveWheelFinalPosition,
                driveWheelFinalSpeed,
                getDriveEncoderUnGearedPosition(),
                getDriveEncoderUnGearedSpeed());

        driveMotorAppliedVoltage = SimulatedBattery.clamp(driveMotorAppliedVoltage);

        /* calculate the stator current */
        driveMotorStatorCurrent = config.driveMotorConfigs.calculateCurrent(driveWheelFinalSpeed, driveMotorAppliedVoltage);

        /* calculate the torque generated */
        Torque driveWheelTorque = config.driveMotorConfigs.calculateTorque(driveMotorStatorCurrent);

        /* calculates the torque if you included losses from friction */
        Torque driveWheelTorqueWithFriction = NewtonMeters.of(MathUtil.applyDeadband(
                driveWheelTorque.in(NewtonMeters),
                config.driveMotorConfigs.friction.in(NewtonMeters),
                Double.POSITIVE_INFINITY));
        return driveWheelTorqueWithFriction.in(NewtonMeters);
    }

    /** @return the current module state of this simulation module */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            MetersPerSecond.of(getDriveWheelFinalSpeed().in(RadiansPerSecond) * config.WHEEL_RADIUS.in(Meters)),
            getSteerAbsoluteFacing());
    }

    /**
     * <h2>Obtains the "free spin" state of the module</h2>
     *
     * <p>The "free spin" state of a simulated module refers to its state after spinning freely for a long time under
     * the current input voltage
     *
     * @return the free spinning module state
     */
    protected SwerveModuleState getFreeSpinState() {
        return new SwerveModuleState(
            config.driveMotorConfigs
                .calculateMechanismVelocity(
                    config.driveMotorConfigs.calculateCurrent(config.driveMotorConfigs.friction),
                    driveMotorAppliedVoltage)
                .in(RadiansPerSecond)
                * config.WHEEL_RADIUS.in(Meters),
            getSteerAbsoluteFacing());
    }

    /**
     * <h2>Cache the encoder values for high-frequency odometry.</h2>
     *
     * <p>An internal method to cache the encoder values to their queues.
     */
    private void updateEncoderCaches() {
        /* Increment of drive wheel position */
        this.driveWheelFinalPosition = this.driveWheelFinalPosition.plus(this.driveWheelFinalSpeed.times(Simulation.simulationDt));

        /* cache sensor readings to queue for high-frequency odometry */
        this.steerAbsolutePositionCache.poll();
        this.steerAbsolutePositionCache.offer(getSteerAbsoluteFacing());

        this.driveWheelFinalPositionCache.poll();
        this.driveWheelFinalPositionCache.offer(driveWheelFinalPosition);
    }

    /**
     * <h2>Obtains the Amount of Current Supplied to the Drive Motor.</h2>
     *
     * @return the current supplied to the drive motor
     */
    public Current getDriveMotorSupplyCurrent() {
        return getDriveMotorStatorCurrent().times(driveMotorAppliedVoltage.div(SimulatedBattery.getBatteryVoltage()));
    }

    /**
     * <h2>Obtains the Stator current the Drive Motor.</h2>
     *
     * @return the stator current of the drive motor
     */
    public Current getDriveMotorStatorCurrent() {
        return driveMotorStatorCurrent;
    }

    /**
     * <h2>Obtains the Position of the Drive Encoder.</h2>
     *
     * <p>This value represents the un-geared position of the encoder, i.e., the amount of radians the drive motor's
     * encoder has rotated.
     *
     * @return the position of the drive motor's encoder (un-geared)
     */
    public Angle getDriveEncoderUnGearedPosition() {
        return getDriveWheelFinalPosition().times(config.DRIVE_GEAR_RATIO);
    }

    /**
     * <h2>Obtains the Final Position of the Wheel.</h2>
     *
     * <p>This method provides the final position of the drive encoder in terms of wheel angle.
     *
     * @return the final position of the drive encoder (wheel rotations)
     */
    public Angle getDriveWheelFinalPosition() {
        return driveWheelFinalPosition;
    }

    /**
     * <h2>Obtains the Speed of the Drive Encoder.</h2>
     *
     * @return the un-geared speed of the drive encoder
     */
    public AngularVelocity getDriveEncoderUnGearedSpeed() {
        return getDriveWheelFinalSpeed().times(config.DRIVE_GEAR_RATIO);
    }

    /**
     * <h2>Obtains the Final Speed of the Wheel.</h2>
     *
     * @return the final speed of the drive wheel
     */
    public AngularVelocity getDriveWheelFinalSpeed() {
        return driveWheelFinalSpeed;
    }

    /**
     * <h2>Obtains the Relative Position of the Steer Encoder.</h2>
     *
     * @return the relative encoder position of the steer motor
     * @see SimulatedMotor#getEncoderPosition()
     */
    public Angle getSteerRelativeEncoderPosition() {
        return getSteerAbsoluteFacing()
            .getMeasure()
            .times(config.STEER_GEAR_RATIO)
            .plus(steerRelativeEncoderOffSet);
    }

    /**
     * <h2>Obtains the Speed of the Steer Relative Encoder (Geared).</h2>
     *
     * @return the speed of the steer relative encoder
     * @see SimulatedMotor#getEncoderVelocity()
     */
    public AngularVelocity getSteerRelativeEncoderVelocity() {
        return getSteerAbsoluteEncoderSpeed().times(config.STEER_GEAR_RATIO);
    }

    /**
     * <h2>Obtains the Absolute Facing of the Steer Mechanism.</h2>
     *
     * @return the absolute facing of the steer mechanism, as a {@link Rotation2d}
     */
    public Rotation2d getSteerAbsoluteFacing() {
        return new Rotation2d(getSteerAbsoluteAngle());
    }

    /**
     * <h2>Obtains the Absolute Angle of the Steer Mechanism.</h2>
     *
     * @return the (continuous) final angle of the steer mechanism, as a {@link Angle}
     * @see SimulatedMotor#getAngularPosition()
     */
    public Angle getSteerAbsoluteAngle() {
        return steerMotorSim.getAngularPosition();
    }

    /**
     * <h2>Obtains the Absolute Rotational Velocity of the Steer Mechanism.</h2>
     *
     * @return the absolute angular velocity of the steer mechanism
     */
    public AngularVelocity getSteerAbsoluteEncoderSpeed() {
        return steerMotorSim.getVelocity();
    }

    /**
     * <h2>Obtains the Cached Readings of the Drive Encoder's Un-Geared Position.</h2>
     *
     * <p>The values of {@link #getDriveEncoderUnGearedPosition()} are cached at each sub-tick and can be retrieved
     * using this method.
     *
     * @return an array of cached drive encoder un-geared positions
     */
    public Angle[] getCachedDriveEncoderUnGearedPositions() {
        return driveWheelFinalPositionCache.stream()
                .map(value -> value.times(config.DRIVE_GEAR_RATIO))
                .toArray(Angle[]::new);
    }

    /**
     * <h2>Obtains the Cached Readings of the Drive Encoder's Final Position (Wheel Rotations).</h2>
     *
     * <p>The values of {@link #getDriveWheelFinalPosition()} are cached at each sub-tick and are divided by the gear
     * ratio to obtain the final wheel rotations.
     *
     * @return an array of cached drive encoder final positions (wheel rotations)
     */
    public Angle[] getCachedDriveWheelFinalPositions() {
        return driveWheelFinalPositionCache.toArray(Angle[]::new);
    }

    /**
     * <h2>Obtains the Cached Readings of the Steer Relative Encoder's Position.</h2>
     *
     * <p>The values of {@link #getSteerRelativeEncoderPosition()} are cached at each sub-tick and can be retrieved
     * using this method.
     *
     * @return an array of cached steer relative encoder positions
     */
    public Angle[] getCachedSteerRelativeEncoderPositions() {
        return steerAbsolutePositionCache.stream()
            .map(absoluteFacing -> absoluteFacing
                .getMeasure()
                .times(config.STEER_GEAR_RATIO)
                .plus(steerRelativeEncoderOffSet))
            .toArray(Angle[]::new);
    }

    /**
     * <h2>Obtains the Cached Readings of the Steer Absolute Positions.</h2>
     *
     * <p>The values of {@link #getSteerAbsoluteFacing()} are cached at each sub-tick and can be retrieved using this
     * method.
     *
     * @return an array of cached absolute steer positions, as {@link Rotation2d} objects
     */
    public Rotation2d[] getCachedSteerAbsolutePositions() {
        return steerAbsolutePositionCache.toArray(Rotation2d[]::new);
    }
}