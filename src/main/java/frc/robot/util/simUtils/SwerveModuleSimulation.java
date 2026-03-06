package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.util.simUtils.SimulatedMotor.SimMotorConfigs;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

public class SwerveModuleSimulation {
    public static class SwerveModuleSimulationConfig implements Supplier<SwerveModuleSimulation> {
        public final SimMotorConfigs driveMotorConfigs, steerMotorConfigs;
        public final double driveGearRatio;
        public final double steerGearRatio;
        public final double wheelCOF;
        public final Voltage driveFrictionVoltage;
        public final Distance wheelRadius;

        /**
         * Configuration for Swerve Module Simulation.
         * If using custom timings, call SimulatedArena.overrideSimulationTimings before constructing modules.
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
            this.driveGearRatio = driveGearRatio;
            this.steerGearRatio = steerGearRatio;
            wheelCOF = wheelsCoefficientOfFriction;
            this.driveFrictionVoltage = driveFrictionVoltage;
            this.wheelRadius = wheelRadius;
        }

        @Override
        public SwerveModuleSimulation get() {
            return new SwerveModuleSimulation(this);
        }

        public double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {
            return gravityForceOnModuleNewtons * wheelCOF;
        }

        /**
         * Returns theoretical max ground speed (m/s).
         */
        public LinearVelocity maximumGroundSpeed() {
            return MetersPerSecond.of(
                    driveMotorConfigs.freeSpinMechanismVelocity().in(RadiansPerSecond) * wheelRadius.in(Meters));
        }

        /**
         * Returns theoretical max propelling force per module.
         * Considers both motor torque and wheel grip.
         */
        public Force getTheoreticalPropellingForcePerModule(Mass robotMass, int modulesCount, Current statorCurrentLimit) {
            final double
                    maxThrustNewtons =
                            driveMotorConfigs.calculateTorque(statorCurrentLimit).in(NewtonMeters)
                                    / wheelRadius.in(Meters),
                    maxGrippingNewtons = 9.8 * robotMass.in(Kilograms) / modulesCount * wheelCOF;

            return Newtons.of(Math.min(maxThrustNewtons, maxGrippingNewtons));
        }

        /**
         * Returns theoretical max linear acceleration for the robot.
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
     * Constructs a Swerve Module Simulation.
     * If using custom timings, call SimulatedArena.overrideSimulationTimings before constructing modules.
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
     * Sets the drive motor controller.
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
     * Sets the steer motor controller.
     */
    public <T extends SimulatedMotorController> T useSteerMotorController(T steerMotorController) {
        return this.steerMotorSim.useMotorController(steerMotorController);
    }

    public SimulatedMotorController.GenericMotorController useGenericControllerForSteer() {
        return this.steerMotorSim.useSimpleDCMotorController();
    }

    /**
     * Updates the simulation for this module.
     * Returns the propelling force vector.
     */
    public Translation2d updateSimulationSubTickGetModuleForce(
        Translation2d moduleCurrentGroundVelocityWorldRelative,
        Rotation2d robotFacing,
        double gravityForceOnModuleNewtons
    ) {
        steerMotorSim.update(Simulation.simulationDt);

        final double grippingForceNewtons = config.getGrippingForceNewtons(gravityForceOnModuleNewtons);
        final Rotation2d moduleWorldFacing = this.getSteerAbsoluteFacing().plus(robotFacing);
        final Translation2d propellingForce = getPropellingForce(grippingForceNewtons, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);

        updateEncoderCaches();

        return propellingForce;
    }

    /**
     * Calculates the propelling force generated by the module.
     * Hybrid approach: Kinematics provides desired velocity, but actual wheel speed
     * and force are constrained by motor capability and grip.
     * 
     * The motor tries to reach the desired speed, generating whatever torque/force
     * it can (clamped to grip limit). The actual wheel speed is what the motor
     * produces, not the kinematic ideal.
     */
    private Translation2d getPropellingForce(double grippingForceNewtons, Rotation2d moduleWorldFacing, Translation2d moduleDesiredGroundVelocity) {
        // Kinematics provides the desired direction and speed at the module
        final double desiredSpeedMPS = moduleDesiredGroundVelocity.getNorm();
        
        // Project desired velocity onto the wheel's rolling direction
        final double desiredSpeedAlignedWithWheelMPS = desiredSpeedMPS
            * moduleDesiredGroundVelocity.getAngle().minus(moduleWorldFacing).getCos();

        // Store the desired speed for the motor controller to use as a setpoint
        final AngularVelocity desiredWheelSpeed = 
            RadiansPerSecond.of(desiredSpeedAlignedWithWheelMPS / config.wheelRadius.in(Meters));
        
        // The motor controller tries to reach this desired speed
        // It generates torque based on current position/speed vs desired speed
        final double driveWheelTorque = getDriveWheelTorque();
        double propellingForceNewtons = driveWheelTorque / config.wheelRadius.in(Meters);
        
        // The actual force is limited by wheel grip (no slip constraint)
        // If motor tries to push harder than the wheel can grip, it slips
        if(Math.abs(propellingForceNewtons) > grippingForceNewtons) {
            // Wheel is slipping - limit force to grip maximum
            propellingForceNewtons = Math.copySign(grippingForceNewtons, propellingForceNewtons);
            
            // When slipping, wheel speed is determined by slip, not the motor control
            // The wheel speed approaches the equilibrium between motor torque and grip
            final AngularVelocity motorEquilibriumSpeed = config.driveMotorConfigs.calculateMechanismVelocity(
                config.driveMotorConfigs.calculateCurrent(
                    NewtonMeters.of(propellingForceNewtons * config.wheelRadius.in(Meters))),
                driveMotorAppliedVoltage);
            
            // Blend actual speed toward motor equilibrium when slipping
            this.driveWheelFinalSpeed = driveWheelFinalSpeed.times(0.8).plus(motorEquilibriumSpeed.times(0.2));
        } else {
            // Not slipping - wheel is actually moving with the ground
            // Actual wheel speed evolves toward desired speed (motor control response)
            // This represents the motor controller's acceleration response
            final double speedError = desiredWheelSpeed.in(RadiansPerSecond) - driveWheelFinalSpeed.in(RadiansPerSecond);
            final double maxAccel = 50.0; // rad/s² - represents motor control response rate
            final double accelLimited = Math.max(-maxAccel, Math.min(maxAccel, speedError * 20)); // Proportional control
            
            this.driveWheelFinalSpeed = RadiansPerSecond.of(
                driveWheelFinalSpeed.in(RadiansPerSecond) + accelLimited * Simulation.simulationDtSeconds
            );
        }

        return new Translation2d(propellingForceNewtons, moduleWorldFacing);
    }

    /**
     * Calculates the torque the drive motor can generate on the wheel.
     */
    private double getDriveWheelTorque() {
        driveMotorAppliedVoltage = driveMotorController.updateControlSignal(
                driveWheelFinalPosition,
                driveWheelFinalSpeed,
                getDriveEncoderUnGearedPosition(),
                getDriveEncoderUnGearedSpeed());

        driveMotorAppliedVoltage = SimulatedBattery.clamp(driveMotorAppliedVoltage);
        driveMotorStatorCurrent = config.driveMotorConfigs.calculateCurrent(driveWheelFinalSpeed, driveMotorAppliedVoltage);

        Torque driveWheelTorque = config.driveMotorConfigs.calculateTorque(driveMotorStatorCurrent);
        Torque driveWheelTorqueWithFriction = NewtonMeters.of(MathUtil.applyDeadband(
                driveWheelTorque.in(NewtonMeters),
                config.driveMotorConfigs.friction.in(NewtonMeters),
                Double.POSITIVE_INFINITY));
        return driveWheelTorqueWithFriction.in(NewtonMeters);
    }

    /** Returns the current module state. */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            MetersPerSecond.of(getDriveWheelFinalSpeed().in(RadiansPerSecond) * config.wheelRadius.in(Meters)),
            getSteerAbsoluteFacing());
    }

    /**
     * Returns the "free spin" state of the module.
     * This is the state after spinning freely for a long time under current voltage.
     */
    protected SwerveModuleState getFreeSpinState() {
        return new SwerveModuleState(
            config.driveMotorConfigs
                .calculateMechanismVelocity(
                    config.driveMotorConfigs.calculateCurrent(config.driveMotorConfigs.friction),
                    driveMotorAppliedVoltage)
                .in(RadiansPerSecond)
                * config.wheelRadius.in(Meters),
            getSteerAbsoluteFacing());
    }

    /**
     * Caches encoder values for high-frequency odometry.
     */
    private void updateEncoderCaches() {
        this.driveWheelFinalPosition = this.driveWheelFinalPosition.plus(this.driveWheelFinalSpeed.times(Simulation.simulationDt));

        this.steerAbsolutePositionCache.poll();
        this.steerAbsolutePositionCache.offer(getSteerAbsoluteFacing());

        this.driveWheelFinalPositionCache.poll();
        this.driveWheelFinalPositionCache.offer(driveWheelFinalPosition);
    }

    /**
     * Returns the current supplied to the drive motor.
     */
    public Current getDriveMotorSupplyCurrent() {
        return getDriveMotorStatorCurrent().times(driveMotorAppliedVoltage.div(SimulatedBattery.getBatteryVoltage()));
    }

    /**
     * Returns the stator current of the drive motor.
     */
    public Current getDriveMotorStatorCurrent() {
        return driveMotorStatorCurrent;
    }

    /**
     * Returns the un-geared position of the drive encoder (motor-side, radians).
     */
    public Angle getDriveEncoderUnGearedPosition() {
        return getDriveWheelFinalPosition().times(config.driveGearRatio);
    }

    /**
     * Returns the final position of the drive wheel (wheel-side, radians).
     */
    public Angle getDriveWheelFinalPosition() {
        return driveWheelFinalPosition;
    }

    /**
     * Returns the un-geared speed of the drive encoder (motor-side).
     */
    public AngularVelocity getDriveEncoderUnGearedSpeed() {
        return getDriveWheelFinalSpeed().times(config.driveGearRatio);
    }

    /**
     * Returns the final speed of the drive wheel (wheel-side).
     */
    public AngularVelocity getDriveWheelFinalSpeed() {
        return driveWheelFinalSpeed;
    }

    /**
     * Returns the relative position of the steer encoder (geared).
     */
    public Angle getSteerRelativeEncoderPosition() {
        return getSteerAbsoluteFacing()
            .getMeasure()
            .times(config.steerGearRatio)
            .plus(steerRelativeEncoderOffSet);
    }

    /**
     * Returns the speed of the steer relative encoder (geared).
     */
    public AngularVelocity getSteerRelativeEncoderVelocity() {
        return getSteerAbsoluteEncoderSpeed().times(config.steerGearRatio);
    }

    /**
     * Returns the absolute facing of the steer mechanism.
     */
    public Rotation2d getSteerAbsoluteFacing() {
        return new Rotation2d(getSteerAbsoluteAngle());
    }

    /**
     * Returns the absolute angle of the steer mechanism.
     */
    public Angle getSteerAbsoluteAngle() {
        return steerMotorSim.getAngularPosition();
    }

    /**
     * Returns the absolute angular velocity of the steer mechanism.
     */
    public AngularVelocity getSteerAbsoluteEncoderSpeed() {
        return steerMotorSim.getVelocity();
    }

    /**
     * Returns cached drive encoder un-geared positions (motor-side).
     */
    public Angle[] getCachedDriveEncoderUnGearedPositions() {
        return driveWheelFinalPositionCache.stream()
                .map(value -> value.times(config.driveGearRatio))
                .toArray(Angle[]::new);
    }

    /**
     * Returns cached drive wheel final positions (wheel-side).
     */
    public Angle[] getCachedDriveWheelFinalPositions() {
        return driveWheelFinalPositionCache.toArray(Angle[]::new);
    }

    /**
     * Returns cached steer relative encoder positions (geared).
     */
    public Angle[] getCachedSteerRelativeEncoderPositions() {
        return steerAbsolutePositionCache.stream()
            .map(absoluteFacing -> absoluteFacing
                .getMeasure()
                .times(config.steerGearRatio)
                .plus(steerRelativeEncoderOffSet))
            .toArray(Angle[]::new);
    }

    /**
     * Returns cached absolute steer positions.
     */
    public Rotation2d[] getCachedSteerAbsolutePositions() {
        return steerAbsolutePositionCache.toArray(Rotation2d[]::new);
    }
}
