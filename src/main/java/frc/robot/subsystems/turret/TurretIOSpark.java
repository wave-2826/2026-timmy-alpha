package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.SparkUtil;

import static frc.robot.util.SparkUtil.tryUntilOk;

import static frc.robot.util.SparkUtil.checkFault;
import static frc.robot.util.SparkUtil.getIfOk;

public class TurretIOSpark implements TurretIO {
    private static final boolean DISABLE_AZIMUTH_ABS_ENCODER = true;

    public final SparkFlex topFlywheelMotor    = new SparkFlex(topFlywheelCanID, MotorType.kBrushless);
    public final SparkFlex bottomFlywheelMotor = new SparkFlex(bottomFlywheelCanID, MotorType.kBrushless);
    public final SparkFlex azimuthMotor        = new SparkFlex(azimuthCanID, MotorType.kBrushless);
    public final SparkFlex hoodMotor           = new SparkFlex(hoodCanID, MotorType.kBrushless);

    public final SparkClosedLoopController flywheelController = topFlywheelMotor.getClosedLoopController();
    public final SparkClosedLoopController azimuthController = azimuthMotor.getClosedLoopController();
    public final SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();

    protected final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    protected final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
    protected final RelativeEncoder azimuthEncoder = azimuthMotor.getEncoder();
    protected final SparkAbsoluteEncoder azimuthAbsEncoder = azimuthMotor.getAbsoluteEncoder();
    protected final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    public TurretIOSpark() {
        configureAndReset();
    }

    public void configureAndReset() {
        // NOTE: do NOT turn on voltage compensation for these motors. Because
        // we manually calculate BackEMF voltage plus our control signal voltage, but
        // only one of those - control signal - must actually scale by a voltage compensation
        // factor because it is used through duty cycle.

        // Flywheel motors
        var flywheelBaseConfig = new SparkFlexConfig();
        flywheelBaseConfig.signals.apply(SparkUtil.defaultSignals);
        flywheelBaseConfig.signals.iAccumulationAlwaysOn(true).iAccumulationPeriodMs(100);
        TurretConstants.flywheelMotorPID.applyConfigAndRegister(flywheelBaseConfig, topFlywheelMotor, bottomFlywheelMotor);
        flywheelBaseConfig.closedLoopRampRate(1.0);
        flywheelBaseConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(flywheelCurrentLimit);
        flywheelBaseConfig.encoder
            .positionConversionFactor(2.0 * Math.PI) // rotations -> radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0) // RPM -> rad/s
            .uvwAverageDepth(2)
            .uvwMeasurementPeriod(16);
        var topFlywheelConfig = new SparkFlexConfig().apply(flywheelBaseConfig);
        var bottomFlywheelConfig = new SparkFlexConfig().apply(flywheelBaseConfig);
        bottomFlywheelConfig.follow(topFlywheelMotor, true);
        tryUntilOk(topFlywheelMotor, 5, () -> topFlywheelMotor.configure(topFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(bottomFlywheelMotor, 5, () -> bottomFlywheelMotor.configure(bottomFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Azimuth motor
        var azimuthConfig = new SparkFlexConfig();
        azimuthConfig.signals.apply(SparkUtil.defaultSignals);
        azimuthConfig.signals
            .absoluteEncoderPositionAlwaysOn(true).absoluteEncoderPositionPeriodMs(50)
            .absoluteEncoderVelocityAlwaysOn(true).absoluteEncoderVelocityPeriodMs(50)
            .iAccumulationAlwaysOn(true).iAccumulationPeriodMs(100);
        azimuthMotorPID.applyConfigAndRegister(azimuthConfig, azimuthMotor);
        azimuthConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2.0 * Math.PI / TurretConstants.totalAzimuthGearing);
        azimuthConfig.absoluteEncoder
            .zeroOffset(0)
            .zeroCentered(false)
            .positionConversionFactor(2.0 * Math.PI) // Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0); // RPM -> rad/s (of ring)
        azimuthConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(azimuthCurrentLimit);
        azimuthConfig.encoder
            .positionConversionFactor(2.0 * Math.PI) // Rotor Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0); // RPM -> rad/s (of ring)
        tryUntilOk(azimuthMotor, 5, () -> azimuthMotor.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Hood motor
        var hoodConfig = new SparkFlexConfig();
        hoodConfig.signals.apply(SparkUtil.defaultSignals);
        hoodConfig.signals.iAccumulationAlwaysOn(true).iAccumulationPeriodMs(100);
        hoodMotorPID.applyConfigAndRegister(hoodConfig, hoodMotor);
        hoodConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(hoodCurrentLimit);
        hoodConfig.encoder
            .positionConversionFactor(2.0 * Math.PI) // Rotor Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0); // RPM -> rad/s (of ring)
        tryUntilOk(hoodMotor, 5, () -> hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        hoodEncoder.setPosition(azimuthEncoder.getPosition());
    }
  
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        var topFlywheelVelocity = getIfOk(topFlywheelMotor, topFlywheelEncoder::getVelocity, 0);
        var topFlywheelCurrent = getIfOk(topFlywheelMotor, topFlywheelMotor::getOutputCurrent, 0);
        inputs.topFlywheel = new TurretIOInputs.FlywheelMotorInputs(
            !checkFault(),
            topFlywheelVelocity, topFlywheelCurrent
        );
        
        var bottomFlywheelVelocity = getIfOk(bottomFlywheelMotor, bottomFlywheelEncoder::getVelocity, 0);
        var bottomFlywheelCurrent = getIfOk(bottomFlywheelMotor, bottomFlywheelMotor::getOutputCurrent, 0);
        inputs.bottomFlywheel = new TurretIOInputs.FlywheelMotorInputs(
            !checkFault(),
            bottomFlywheelVelocity, bottomFlywheelCurrent
        );

        var azimuthInternalAngle = getIfOk(azimuthMotor, azimuthEncoder::getPosition, 0);
        var azimuthInternalVelocity = getIfOk(azimuthMotor, azimuthEncoder::getVelocity, 0);
        
        double azimuthAngle, azimuthVelocity;
        if(DISABLE_AZIMUTH_ABS_ENCODER) {
            azimuthAngle = azimuthInternalAngle;
            azimuthVelocity = azimuthInternalVelocity;
        } else {
            azimuthAngle = getIfOk(azimuthMotor, azimuthAbsEncoder::getPosition, 0) / TurretConstants.totalAzimuthGearing;
            azimuthVelocity = getIfOk(azimuthMotor, azimuthAbsEncoder::getVelocity, 0) / TurretConstants.totalAzimuthGearing;
        }

        var azimuthCurrent = getIfOk(azimuthMotor, azimuthMotor::getOutputCurrent, 0);
        inputs.azimuth = new TurretIOInputs.AzimuthMotorInputs(
            !checkFault(),
            azimuthInternalAngle, azimuthInternalVelocity,
            azimuthCurrent
        );
        inputs.azimuthEncoder = new TurretIOInputs.AzimuthEncoderInputs(true, azimuthAngle, azimuthVelocity);

        var hoodAngle = getIfOk(hoodMotor, hoodEncoder::getPosition, 0);
        var hoodVelocity = getIfOk(hoodMotor, hoodEncoder::getVelocity, 0);
        var hoodCurrent = getIfOk(hoodMotor, hoodMotor::getOutputCurrent, 0);
        inputs.hood = new TurretIOInputs.HoodMotorInputs(
            !checkFault(),
            hoodAngle, hoodVelocity,
            hoodCurrent
        );
    }

    @Override
    public void setPIDOutputs(TurretIOPIDOutputs outputs) {
        var flyMotorSetpoint = outputs.flywheelSpeedRadPerSec() / TurretConstants.totalFlywheelGearing;
        // TODO: Calculate next velocity
        var ff = flywheelMotorFF.calculateWithVelocities(flyMotorSetpoint, flyMotorSetpoint);
        flywheelController.setSetpoint(flyMotorSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
        azimuthController.setSetpoint(outputs.azimuthAngleRad() / TurretConstants.totalAzimuthGearing, ControlType.kPosition);
        hoodController.setSetpoint((
            outputs.hoodAngleRad() / TurretConstants.hoodRingToHoodReduction -
            outputs.azimuthAngleRad()
        ) * TurretConstants.hoodMotorToRingReduction, ControlType.kPosition);
    }

    @Override
    public void setLQROutputs(TurretLQROutputs outputs) {
        flywheelController.setSetpoint(outputs.flywheelCurrent(), ControlType.kCurrent, ClosedLoopSlot.kSlot1);
        azimuthController.setSetpoint(outputs.azimuthCurrent(), ControlType.kCurrent, ClosedLoopSlot.kSlot1);
        hoodController.setSetpoint(outputs.hoodCurrent(), ControlType.kCurrent, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void stop() {
        topFlywheelMotor.stopMotor();
        bottomFlywheelMotor.stopMotor();
        azimuthMotor.stopMotor();
        hoodMotor.stopMotor();
    }
}
