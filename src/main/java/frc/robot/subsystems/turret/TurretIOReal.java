package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;
import frc.robot.util.SparkUtil;

import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.checkFault;
import static frc.robot.util.SparkUtil.getIfOk;

public class TurretIOReal implements TurretIO {
    protected final SparkFlex topFlywheelMotor    = new SparkFlex(topFlywheelCanID, MotorType.kBrushless);
    protected final SparkFlex bottomFlywheelMotor = new SparkFlex(bottomFlywheelCanID, MotorType.kBrushless);
    protected final SparkMax azimuthMotor         = new SparkMax(azimuthCanID, MotorType.kBrushless);
    protected final SparkFlex hoodMotor           = new SparkFlex(hoodCanID, MotorType.kBrushless);

    protected final SparkClosedLoopController flywheelController;
    protected final SparkClosedLoopController azimuthController;
    protected final SparkClosedLoopController hoodController;

    protected final RelativeEncoder topFlywheelEncoder = topFlywheelMotor.getEncoder();
    protected final RelativeEncoder bottomFlywheelEncoder = bottomFlywheelMotor.getEncoder();
    protected final RelativeEncoder azimuthEncoder = azimuthMotor.getEncoder();
    protected final SparkAbsoluteEncoder azimuthAbsEncoder = azimuthMotor.getAbsoluteEncoder();
    protected final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    public TurretIOReal() {
        // Flywheel motors
        var flywheelBaseConfig = new SparkFlexConfig();
        flywheelBaseConfig.signals.apply(SparkUtil.defaultSignals);
        TurretConstants.flywheelMotorPID.applyConfigAndRegister(flywheelBaseConfig, topFlywheelMotor, bottomFlywheelMotor);
        flywheelBaseConfig.closedLoopRampRate(1.0);
        flywheelBaseConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(flywheelCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        flywheelBaseConfig.encoder
            .positionConversionFactor(2.0 * Math.PI) // rotations -> radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0); // RPM -> rad/s
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
            .absoluteEncoderVelocityAlwaysOn(true).absoluteEncoderVelocityPeriodMs(50);
        azimuthMotorPID.applyConfigAndRegister(azimuthConfig, azimuthMotor);
        azimuthConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, Math.PI * 2);
        azimuthConfig.absoluteEncoder
            .zeroOffset(0)
            .zeroCentered(false)
            .positionConversionFactor(2.0 * Math.PI) // Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0); // RPM -> rad/s (of ring)
        azimuthConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(azimuthCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        azimuthConfig.encoder
            .positionConversionFactor(2.0 * Math.PI * azimuthToRingReduction) // Rotor Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 * azimuthToRingReduction); // RPM -> rad/s (of ring)
        tryUntilOk(azimuthMotor, 5, () -> azimuthMotor.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Hood motor
        var hoodConfig = new SparkFlexConfig();
        hoodConfig.signals.apply(SparkUtil.defaultSignals);
        hoodMotorPID.applyConfigAndRegister(hoodConfig, hoodMotor);
        hoodConfig.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, Math.PI * 2);
        hoodConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(hoodCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        hoodConfig.encoder
            .positionConversionFactor(2.0 * Math.PI * hoodToRingReduction) // Rotor Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 * hoodToRingReduction); // RPM -> rad/s (of ring)
        tryUntilOk(hoodMotor, 5, () -> hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Interfaces
        flywheelController = topFlywheelMotor.getClosedLoopController();
        azimuthController = azimuthMotor.getClosedLoopController();
        hoodController = hoodMotor.getClosedLoopController();

        hoodEncoder.setPosition(azimuthEncoder.getPosition());
    }
  
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        var topFlywheelVelocity = getIfOk(topFlywheelMotor, topFlywheelEncoder::getVelocity, 0);
        var topFlywheelCurrent = getIfOk(topFlywheelMotor, topFlywheelMotor::getOutputCurrent, 0);
        inputs.topFlywheel = new TurretIOInputs.FlywheelMotorInputs(checkFault(), topFlywheelVelocity, topFlywheelCurrent);
        
        var bottomFlywheelVelocity = getIfOk(bottomFlywheelMotor, bottomFlywheelEncoder::getVelocity, 0);
        var bottomFlywheelCurrent = getIfOk(bottomFlywheelMotor, bottomFlywheelMotor::getOutputCurrent, 0);
        inputs.bottomFlywheel = new TurretIOInputs.FlywheelMotorInputs(checkFault(), bottomFlywheelVelocity, bottomFlywheelCurrent);

        var azimuthAngle = getIfOk(azimuthMotor, azimuthAbsEncoder::getPosition, 0);
        var azimuthInternalAngle = getIfOk(azimuthMotor, azimuthEncoder::getPosition, 0);
        var azimuthVelocity = getIfOk(azimuthMotor, azimuthAbsEncoder::getVelocity, 0);
        var azimuthCurrent = getIfOk(azimuthMotor, azimuthMotor::getOutputCurrent, 0);
        var azimuthApplied = getIfOk(azimuthMotor, azimuthMotor::getAppliedOutput, 0);
        inputs.azimuth = new TurretIOInputs.AzimuthMotorInputs(checkFault(), azimuthAngle, azimuthInternalAngle, azimuthVelocity, azimuthCurrent, azimuthApplied);

        var hoodRingAngle = getIfOk(hoodMotor, hoodEncoder::getPosition, 0);
        var hoodRingVelocity = getIfOk(hoodMotor, hoodEncoder::getVelocity, 0);
        var hoodCurrent = getIfOk(hoodMotor, hoodMotor::getOutputCurrent, 0);
        var hoodApplied = getIfOk(hoodMotor, hoodMotor::getAppliedOutput, 0);
        inputs.hood = new TurretIOInputs.HoodMotorInputs(checkFault(), hoodRingAngle, hoodRingVelocity, hoodCurrent, hoodApplied);
    }

    @Override
    public void setOutputs(TurretIOOutputs outputs) {
        // TODO: Calculate next velocity
        var ff = flywheelMotorFF.calculateWithVelocities(outputs.flywheelSpeedRadPerSec(), outputs.flywheelSpeedRadPerSec());
        flywheelController.setSetpoint(outputs.flywheelSpeedRadPerSec(), ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
        azimuthController.setSetpoint(outputs.azimuthAngleRad(), ControlType.kPosition);
        hoodController.setSetpoint(outputs.azimuthAngleRad() + outputs.hoodAngleRad(), ControlType.kPosition);
    }

    @Override
    public void stop() {
        flywheelController.setSetpoint(0.0, ControlType.kVelocity);
        azimuthMotor.stopMotor();
        hoodMotor.stopMotor();
    }
}
