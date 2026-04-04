package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

/**
 * + azimuth -> clockwise
 * + hood -> up
 * + flywheel -> outward
 * 
 * azimuth is independent
 * hood and flywheel run based on the difference of their rings to the azimuth ring
 * hood is `([hood ring] - [azimuth ring]) / [hood ring to hood reduction]`
 * therefore [hood ring] is `[azimuth ring] - hood * [hood ring to hood reduction]`
 * 
 */
public class TurretIOTalonFX implements TurretIO {
    protected final Debouncer flywheel1ConnectedDebouncer = new Debouncer(0.5);
    protected final Debouncer flywheel2ConnectedDebouncer = new Debouncer(0.5);
    protected final Debouncer azimuthConnectedDebouncer = new Debouncer(0.5);
    protected final Debouncer hoodConnectedDebouncer = new Debouncer(0.5);

    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUseTimesync(true);
    protected final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true).withUseTimesync(true);
    protected final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true).withUseTimesync(true);
    protected final CoastOut coastRequest = new CoastOut();
    protected final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0).withUseTimesync(true);
    
    protected final Follower followerRequest;

    protected final DigitalInput azimuthZeroSensor = new DigitalInput(TurretConstants.azimuthZeroDIOPort);

    protected final TalonFX flywheel1Talon = new TalonFX(TurretConstants.flywheel1CanID, TurretConstants.CANBus);
    protected final TalonFX flywheel2Talon = new TalonFX(TurretConstants.flywheel2CanID, TurretConstants.CANBus);
    protected final TalonFX azimuthTalon = new TalonFX(TurretConstants.azimuthCanID, TurretConstants.CANBus);
    protected final TalonFX hoodTalon = new TalonFX(TurretConstants.hoodCanID, TurretConstants.CANBus);
    // protected final CANcoder azimuthCancoder = new CANcoder(TurretConstants.azimuthCancoderID, TurretConstants.CANBus);
    
    /** Base unit: motor **rotations per second** */
    protected final StatusSignal<AngularVelocity> flywheel1Velocity;
    /** Base unit: stator **amps** */
    protected final StatusSignal<Current> flywheel1Current;
    /** Base unit: motor **rotations per second** */
    protected final StatusSignal<AngularVelocity> flywheel2Velocity;
    /** Base unit: stator **amps** */
    protected final StatusSignal<Current> flywheel2Current;

    /** Base unit: mechanism **rotations** */
    protected final StatusSignal<Angle> azimuthInternalAngle;
    /** Base unit: mechanism **rotations per second** */
    protected final StatusSignal<AngularVelocity> azimuthInternalVelocity;
    /** Base unit: stator **amps** */
    protected final StatusSignal<Current> azimuthCurrent;

    /** Base unit: mechanism **rotations** */
    protected final StatusSignal<Angle> hoodAngle;
    /** Base unit: mechanism **rotations per second** */
    protected final StatusSignal<AngularVelocity> hoodVelocity;
    /** Base unit: stator **amps** */
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

        var flywheelConfig = baseConfig.clone();
        flywheelConfig.Feedback.SensorToMechanismRatio = 1. / TurretConstants.totalFlywheelGearing;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        applyTorqueCurrentLimit(flywheelConfig, TurretConstants.flywheelCurrentLimit);

        TurretConstants.flywheelMotorPID.applyConfigAndRegister(flywheelConfig, flywheel1Talon, flywheel2Talon);

        tryUntilOk(5, () -> flywheel1Talon.getConfigurator().apply(flywheelConfig, 0.25));
        tryUntilOk(5, () -> flywheel2Talon.getConfigurator().apply(flywheelConfig, 0.25));

        var azimuthConfig = baseConfig.clone();
        azimuthConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        applyTorqueCurrentLimit(azimuthConfig, TurretConstants.azimuthCurrentLimit);

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

        followerRequest = new Follower(flywheel1Talon.getDeviceID(), MotorAlignmentValue.Aligned);

        // We just don't configure the CANCoder - configure with Phoenix Tuner instead

        flywheel1Velocity = flywheel1Talon.getVelocity();
        flywheel1Current = flywheel1Talon.getStatorCurrent();
        flywheel2Velocity = flywheel2Talon.getVelocity();
        flywheel2Current = flywheel2Talon.getStatorCurrent();

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
            flywheel1Velocity,
            flywheel2Velocity, flywheel2Current,
            azimuthInternalAngle, azimuthInternalVelocity, azimuthCurrent,
            hoodAngle, hoodVelocity, hoodCurrent);
        
        // Leader update frequency so follower can track more accurately
        flywheel1Current.setUpdateFrequency(250.0);
        ParentDevice.optimizeBusUtilizationForAll(
            flywheel1Talon, flywheel2Talon, azimuthTalon, hoodTalon
        );

        resetAzimuth(Rotation2d.kZero);
        resetHoodTo(TurretConstants.hoodMinAngle);
    }

    @Override
    public void setPIDOutputs(TurretIOPIDOutputs outputs) {
        var flywheel1Connected = flywheel1ConnectedDebouncer.calculate(flywheel1Velocity.getStatus().isOK());
        var velocityReq = flywheelVelocityRequest.withVelocity(outputs.flywheelSpeedRadPerSec() / (2 * Math.PI)).withSlot(0);
        if(flywheel1Connected) {
            if(outputs.flywheelSpeedRadPerSec() < Units.degreesToRadians(10)) {
                flywheel1Talon.setControl(coastRequest);
            } else {
                flywheel1Talon.setControl(velocityReq);
            }
            flywheel2Talon.setControl(followerRequest);
        } else {
            // Fallback for redundancy
            flywheel2Talon.setControl(velocityReq);
        }

        azimuthTalon.setControl(positionRequest.withPosition(
            outputs.azimuthAngleRad() / (2 * Math.PI)
        ).withSlot(0));

        double azimuthRingRotations = azimuthInternalAngle.getValueAsDouble();
        double hoodRingRotations = azimuthRingRotations - outputs.hoodAngleRad() / TurretConstants.hoodRingToHoodReduction / (2 * Math.PI);
        hoodTalon.setControl(positionRequest.withPosition(hoodRingRotations).withSlot(0));
    }
  
    public void updateLQRInputs(TurretIOInputs inputs) {
        BaseStatusSignal.refreshAll(azimuthInternalAngle, azimuthInternalVelocity, flywheel1Velocity, hoodAngle, hoodVelocity);
        inputs.azimuth = new TurretIOInputs.AzimuthMotorInputs(
            inputs.azimuth.connected(),
            azimuthInternalAngle.getValueAsDouble() * (2 * Math.PI),
            azimuthInternalVelocity.getValueAsDouble() * (2 * Math.PI),
            inputs.azimuth.currentAmps()
        );
        inputs.hood = new TurretIOInputs.HoodMotorInputs(
            inputs.hood.connected(),
            hoodAngle.getValueAsDouble() * (2 * Math.PI),
            hoodVelocity.getValueAsDouble() * (2 * Math.PI),
            inputs.hood.currentAmps()
        );
        inputs.flywheel1 = new TurretIOInputs.FlywheelMotorInputs(
            inputs.flywheel1.connected(),
            flywheel1Velocity.getValueAsDouble() * (2 * Math.PI),
            inputs.flywheel1.currentAmps()
        );
        inputs.flywheel2 = new TurretIOInputs.FlywheelMotorInputs(
            inputs.flywheel2.connected(),
            flywheel2Velocity.getValueAsDouble() * (2 * Math.PI),
            inputs.flywheel2.currentAmps()
        );
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        var flywheel1Status = BaseStatusSignal.refreshAll(flywheel1Velocity, flywheel1Current);
        var flywheel2Status = BaseStatusSignal.refreshAll(flywheel2Velocity, flywheel2Current);
        // var azimuthEncoderStatus = BaseStatusSignal.refreshAll(azimuthAbsAngle, azimuthAbsVelocity);
        var azimuthMotorStatus = BaseStatusSignal.refreshAll(azimuthInternalAngle, azimuthInternalVelocity, azimuthCurrent);
        var hoodStatus = BaseStatusSignal.refreshAll(hoodAngle, hoodVelocity, hoodCurrent);
        
        inputs.azimuth = new TurretIOInputs.AzimuthMotorInputs(
            azimuthConnectedDebouncer.calculate(azimuthMotorStatus.isOK()),
            azimuthInternalAngle.getValueAsDouble() * (2 * Math.PI),
            azimuthInternalVelocity.getValueAsDouble() * (2 * Math.PI),
            azimuthCurrent.getValueAsDouble()
        );
        inputs.flywheel1 = new TurretIOInputs.FlywheelMotorInputs(
            flywheel1ConnectedDebouncer.calculate(flywheel1Status.isOK()),
            flywheel1Velocity.getValueAsDouble() * (2 * Math.PI),
            flywheel1Current.getValueAsDouble()
        );
        inputs.flywheel2 = new TurretIOInputs.FlywheelMotorInputs(
            flywheel2ConnectedDebouncer.calculate(flywheel2Status.isOK()),
            flywheel2Velocity.getValueAsDouble() * (2 * Math.PI),
            flywheel2Current.getValueAsDouble()
        );

        inputs.hood = new TurretIOInputs.HoodMotorInputs(
            hoodConnectedDebouncer.calculate(hoodStatus.isOK()),
            hoodAngle.getValueAsDouble() * (2 * Math.PI),
            hoodVelocity.getValueAsDouble() * (2 * Math.PI),
            hoodCurrent.getValueAsDouble()
        );

        inputs.azimuthZeroTriggered = !azimuthZeroSensor.get();
    }

    @Override
    public void setVelocityOutputs(double flywheelVelocityRadPerSec, double azimuthVelocityRadPerSec,
            double hoodVelocityRadPerSec) {
        flywheel1Talon.setControl(velocityRequest.withVelocity(flywheelVelocityRadPerSec / (Math.PI * 2)).withSlot(1));
        flywheel2Talon.setControl(followerRequest);
        azimuthTalon.setControl(velocityRequest.withVelocity(azimuthVelocityRadPerSec / (Math.PI * 2)).withSlot(1));
        hoodTalon.setControl(velocityRequest.withVelocity(hoodVelocityRadPerSec / (Math.PI * 2)).withSlot(1));
    }

    public void setLQROutputs(TurretLQROutputs outputs) {
        flywheel1Talon.setControl(torqueCurrentRequest.withOutput(outputs.flywheelCurrent()));
        flywheel2Talon.setControl(followerRequest);
        azimuthTalon.setControl(torqueCurrentRequest.withOutput(outputs.azimuthCurrent()));
        hoodTalon.setControl(torqueCurrentRequest.withOutput(outputs.hoodCurrent()));
    }

    @Override
    public void resetAzimuth(Rotation2d angle) {
        tryUntilOk(5, () -> azimuthTalon.setPosition(angle.getRotations()));
    }

    @Override
    public void resetHoodTo(double angleRad) {
        // hoodAngle = (azimuth.internalEncoderAngle - hood.angleRad) * TurretConstants.hoodRingToHoodReduction + hoodMinAngle
        // (hoodAngle - hoodMinAngle) / TurretConstants.hoodRingToHoodReduction = azimuth.internalEncoderAngle - hood.angleRad
        // azimuth.internalEncoderAngle - (hoodAngle - hoodMinAngle) / TurretConstants.hoodRingToHoodReduction = hood.angleRad
        double angleRotations = (angleRad - TurretConstants.hoodMinAngle) / 2 / Math.PI;
        tryUntilOk(5, () -> hoodTalon.setPosition(
            azimuthInternalAngle.getValueAsDouble() - angleRotations / TurretConstants.hoodRingToHoodReduction
        ));
    }

    @Override
    public void stop() {
        flywheel1Talon.stopMotor();
        flywheel2Talon.stopMotor();
        azimuthTalon.stopMotor();
        hoodTalon.stopMotor();
    }
}
