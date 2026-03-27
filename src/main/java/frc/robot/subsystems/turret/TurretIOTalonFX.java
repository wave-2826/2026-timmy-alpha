package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class TurretIOTalonFX implements TurretIO {
    private static final boolean DISABLE_AZIMUTH_ABS_ENCODER = true;

    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUseTimesync(true);
    protected final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withUseTimesync(true);
    protected final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true).withUseTimesync(true);
    
    protected final Follower followerRequest;

    protected final TalonFX topFlywheelTalon = new TalonFX(TurretConstants.topFlywheelCanID, TurretConstants.CANBus);
    protected final TalonFX bottomFlywheelTalon = new TalonFX(TurretConstants.bottomFlywheelCanID, TurretConstants.CANBus);
    protected final TalonFX azimuthTalon = new TalonFX(TurretConstants.azimuthCanID, TurretConstants.CANBus);
    protected final TalonFX hoodTalon = new TalonFX(TurretConstants.hoodCanID, TurretConstants.CANBus);
    // protected final CANcoder azimuthCancoder = new CANcoder(TurretConstants.azimuthCancoderID, TurretConstants.CANBus);
    
    protected final StatusSignal<AngularVelocity> topFlywheelVelocity;
    protected final StatusSignal<Current> topFlywheelCurrent;
    protected final StatusSignal<AngularVelocity> bottomFlywheelVelocity;
    protected final StatusSignal<Current> bottomFlywheelCurrent;
    
    // protected final StatusSignal<Angle> azimuthAbsAngle;
    // protected final StatusSignal<AngularVelocity> azimuthAbsVelocity;

    protected final StatusSignal<Angle> azimuthInternalAngle;
    protected final StatusSignal<AngularVelocity> azimuthInternalVelocity;
    protected final StatusSignal<Current> azimuthCurrent;

    protected final StatusSignal<Angle> hoodAngle;
    protected final StatusSignal<AngularVelocity> hoodVelocity;
    protected final StatusSignal<Current> hoodCurrent;

    private void applyTorqueCurrentLimit(TalonFXConfiguration config, double limitAmps) {
        config.TorqueCurrent.PeakForwardTorqueCurrent = limitAmps;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -limitAmps;
        config.CurrentLimits.StatorCurrentLimit = limitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
    }

    public TurretIOTalonFX() {
        var baseConfig = new TalonFXConfiguration();
        baseConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        baseConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Only clockwise motor
        baseConfig.MotorOutput.ControlTimesyncFreqHz = 250;
        applyTorqueCurrentLimit(baseConfig, TurretConstants.flywheelCurrentLimit);

        TurretConstants.flywheelMotorPID.applyConfigAndRegister(baseConfig, topFlywheelTalon, bottomFlywheelTalon);

        tryUntilOk(5, () -> topFlywheelTalon.getConfigurator().apply(baseConfig, 0.25));
        tryUntilOk(5, () -> bottomFlywheelTalon.getConfigurator().apply(baseConfig, 0.25));

        var azimuthConfig = baseConfig.clone();
        azimuthConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        applyTorqueCurrentLimit(azimuthConfig, TurretConstants.azimuthCurrentLimit);

        if(!DISABLE_AZIMUTH_ABS_ENCODER) {
            // TODO: azimuth feedback
        }

        TurretConstants.azimuthMotorPID.applyConfigAndRegister(azimuthConfig, azimuthTalon);
        azimuthConfig.Feedback.SensorToMechanismRatio = 1. / TurretConstants.totalAzimuthGearing;
        azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;

        tryUntilOk(5, () -> azimuthTalon.getConfigurator().apply(azimuthConfig, 0.25));
        
        var hoodConfig = baseConfig.clone();
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        applyTorqueCurrentLimit(hoodConfig, TurretConstants.hoodCurrentLimit);

        TurretConstants.hoodMotorPID.applyConfigAndRegister(hoodConfig, hoodTalon);
        hoodConfig.Feedback.SensorToMechanismRatio = 1. / TurretConstants.hoodMotorToRingReduction;

        tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(hoodConfig, 0.25));

        followerRequest = new Follower(topFlywheelTalon.getDeviceID(), MotorAlignmentValue.Opposed);

        // We just don't configure the CANCoder - configure with Phoenix Tuner instead

        topFlywheelVelocity = topFlywheelTalon.getVelocity();
        topFlywheelCurrent = topFlywheelTalon.getStatorCurrent();
        bottomFlywheelVelocity = bottomFlywheelTalon.getVelocity();
        bottomFlywheelCurrent = bottomFlywheelTalon.getStatorCurrent();

        // azimuthAbsAngle = azimuthCancoder.getAbsolutePosition();
        // azimuthAbsVelocity = azimuthCancoder.getVelocity();

        azimuthInternalAngle = azimuthTalon.getPosition();
        azimuthInternalVelocity = azimuthTalon.getVelocity();
        azimuthCurrent = azimuthTalon.getStatorCurrent();

        hoodAngle = hoodTalon.getPosition();
        hoodVelocity = hoodTalon.getVelocity();
        hoodCurrent = hoodTalon.getStatorCurrent();

        // 50 for all except the leader current
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            topFlywheelVelocity,
            bottomFlywheelVelocity, bottomFlywheelCurrent,
            //azimuthAbsAngle, azimuthAbsVelocity,
            azimuthInternalAngle, azimuthInternalVelocity, azimuthCurrent,
            hoodAngle, hoodVelocity, hoodCurrent);
        // Leader update frequency so follower can track more accurately
        topFlywheelCurrent.setUpdateFrequency(250.0);
        ParentDevice.optimizeBusUtilizationForAll(
            topFlywheelTalon, bottomFlywheelTalon, azimuthTalon, hoodTalon//, azimuthCancoder
        );

        resetAzimuthAndHood();
    }
  
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        var topFlywheelStatus = BaseStatusSignal.refreshAll(topFlywheelVelocity, topFlywheelCurrent);
        var bottomFlywheelStatus = BaseStatusSignal.refreshAll(bottomFlywheelVelocity, bottomFlywheelCurrent);
        // var azimuthEncoderStatus = BaseStatusSignal.refreshAll(azimuthAbsAngle, azimuthAbsVelocity);
        var azimuthMotorStatus = BaseStatusSignal.refreshAll(azimuthInternalAngle, azimuthInternalVelocity, azimuthCurrent);
        var hoodStatus = BaseStatusSignal.refreshAll(hoodAngle, hoodVelocity, hoodCurrent);
        
        inputs.azimuth = new TurretIOInputs.AzimuthMotorInputs(
            azimuthMotorStatus.isOK(),
            azimuthInternalAngle.getValue().in(Radians),
            azimuthInternalVelocity.getValue().in(RadiansPerSecond),
            azimuthCurrent.getValue().in(Amps)
        );
        // inputs.azimuthEncoder = new TurretIOInputs.AzimuthEncoderInputs(
        //     azimuthEncoderStatus.isOK(),
        //     azimuthAbsAngle.getValue().in(Radians),
        //     azimuthAbsVelocity.getValue().in(RadiansPerSecond)
        // );
        inputs.azimuthEncoder = new TurretIOInputs.AzimuthEncoderInputs(false, 0, 0);

        inputs.topFlywheel = new TurretIOInputs.FlywheelMotorInputs(
            topFlywheelStatus.isOK(),
            topFlywheelVelocity.getValue().in(RadiansPerSecond),
            topFlywheelCurrent.getValue().in(Amps)
        );
        inputs.bottomFlywheel = new TurretIOInputs.FlywheelMotorInputs(
            bottomFlywheelStatus.isOK(),
            bottomFlywheelVelocity.getValue().in(RadiansPerSecond),
            bottomFlywheelCurrent.getValue().in(Amps)
        );

        inputs.hood = new TurretIOInputs.HoodMotorInputs(
            hoodStatus.isOK(),
            hoodAngle.getValue().in(Radians),
            hoodVelocity.getValue().in(RadiansPerSecond),
            hoodCurrent.getValue().in(Amps)
        );
    }

    @Override
    public void setPIDOutputs(TurretIOPIDOutputs outputs) {
        topFlywheelTalon.setControl(velocityRequest.withVelocity(
            outputs.flywheelSpeedRadPerSec() / (2 * Math.PI)
        ).withSlot(0));
        bottomFlywheelTalon.setControl(followerRequest);

        azimuthTalon.setControl(positionRequest.withPosition(
            outputs.azimuthAngleRad() / (2 * Math.PI)
        ).withSlot(0));
        double hoodRingPos = azimuthInternalAngle.getValue().in(Radians) + outputs.hoodAngleRad() / TurretConstants.hoodRingToHoodReduction;
        hoodTalon.setControl(positionRequest.withPosition(
            hoodRingPos / (2 * Math.PI)
        ).withSlot(0));
    }

    @Override
    public void setVelocityOutputs(double flywheelVelocityRadPerSec, double azimuthVelocityRadPerSec,
            double hoodVelocityRadPerSec) {
        topFlywheelTalon.setControl(velocityRequest.withVelocity(flywheelVelocityRadPerSec / (Math.PI * 2)).withSlot(1));
        bottomFlywheelTalon.setControl(followerRequest);
        azimuthTalon.setControl(velocityRequest.withVelocity(azimuthVelocityRadPerSec / (Math.PI * 2)).withSlot(1));
        hoodTalon.setControl(velocityRequest.withVelocity(hoodVelocityRadPerSec / (Math.PI * 2)).withSlot(1));
    }

    @Override
    public void setLQROutputs(TurretLQROutputs outputs) {
        topFlywheelTalon.setControl(torqueCurrentRequest.withOutput(outputs.flywheelCurrent()));
        bottomFlywheelTalon.setControl(followerRequest);
        azimuthTalon.setControl(torqueCurrentRequest.withOutput(outputs.azimuthCurrent()));
        hoodTalon.setControl(torqueCurrentRequest.withOutput(outputs.hoodCurrent()));
    }

    @Override
    public void resetAzimuthAndHood() {
        tryUntilOk(5, () -> azimuthTalon.setPosition(0.0));
        tryUntilOk(5, () -> hoodTalon.setPosition(0.0));
    }

    @Override
    public void stop() {
        topFlywheelTalon.stopMotor();
        bottomFlywheelTalon.stopMotor();
        azimuthTalon.stopMotor();
        hoodTalon.stopMotor();
    }
}
