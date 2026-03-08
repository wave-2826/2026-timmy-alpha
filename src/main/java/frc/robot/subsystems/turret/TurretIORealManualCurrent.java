package frc.robot.subsystems.turret;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

/**
 * A slight adjustment to real turret IO wherein we manually control current with an on-rio PID loop
 * instead of using the kCurrent control type; this is far worse and we ideally wouldn't use it, but
 * rev's current control PIDs are seemingly broken in some weird ways??
 */
public class TurretIORealManualCurrent extends TurretIOReal {
    private double manualVoltageCompensation(double voltage) {
        return voltage * 13.4 / RobotController.getBatteryVoltage();
    }
    private double getVoltage(DCMotor motor, double current, double speedRadiansPerSec) {
        return manualVoltageCompensation(1.0 / motor.KvRadPerSecPerVolt * speedRadiansPerSec) +
            motor.rOhms * current;
    }

    public static int fastClosedLoopRate = 100; // hz

    private PIDController flywheelCurrentPID = new PIDController(0, 0, 0, 1. / fastClosedLoopRate);
    private PIDController azimuthCurrentPID = new PIDController(0, 0, 0, 1. / fastClosedLoopRate);
    private PIDController hoodCurrentPID = new PIDController(0, 0, 0, 1. / fastClosedLoopRate);

    public TurretIORealManualCurrent() {
        super();

        TurretConstants.flywheelMotorPID.configureController(flywheelCurrentPID, ClosedLoopSlot.kSlot1);
        TurretConstants.azimuthMotorPID.configureController(azimuthCurrentPID, ClosedLoopSlot.kSlot1);
        TurretConstants.hoodMotorPID.configureController(hoodCurrentPID, ClosedLoopSlot.kSlot1);
    }

    double flySetpoint = 0;
    double aziSetpoint = 0;
    double hoodSetpoint = 0;

    @Override
    public void setLQROutputs(TurretLQROutputs outputs) {
        flySetpoint = outputs.flywheelCurrent();
        aziSetpoint = outputs.azimuthCurrent();
        hoodSetpoint = outputs.hoodCurrent();

        if(!Constants.isSim) runFastClosedLoop();
    }

    public void runFastClosedLoop() {
        var ffFlywheel = getVoltage(TurretConstants.flywheelSimMotor, flySetpoint, topFlywheelEncoder.getVelocity());
        var ffAzimuth = getVoltage(TurretConstants.azimuthSimMotor, aziSetpoint, azimuthEncoder.getVelocity());
        var ffHood = getVoltage(TurretConstants.hoodSimMotor, hoodSetpoint, hoodEncoder.getVelocity());

        var fbFlywheel = flywheelCurrentPID.calculate(topFlywheelMotor.getOutputCurrent(), flySetpoint);
        var fbAzimuth = azimuthCurrentPID.calculate(azimuthMotor.getOutputCurrent(), aziSetpoint);
        var fbHood = hoodCurrentPID.calculate(hoodMotor.getOutputCurrent(), hoodSetpoint);

        flywheelController.setSetpoint(ffFlywheel + fbFlywheel, ControlType.kDutyCycle);
        azimuthController.setSetpoint(ffAzimuth + fbAzimuth, ControlType.kDutyCycle);
        hoodController.setSetpoint(ffHood + fbHood, ControlType.kDutyCycle);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
