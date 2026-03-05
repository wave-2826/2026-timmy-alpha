package frc.robot.subsystems.turret.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.generated.TurretTuningData;

public final class TurretSimDCMotor implements TurretSim {
    DCMotorSim azimuth = new DCMotorSim(LinearSystemId.createDCMotorSystem(
        TurretConstants.azimuthSimMotor, TurretConstants.azimuthMotorInertiaKgM2, 1
    ), TurretConstants.azimuthSimMotor);
    DCMotorSim hood = new DCMotorSim(LinearSystemId.createDCMotorSystem(
        TurretConstants.hoodSimMotor, TurretConstants.hoodMotorInertiaKgM2, 1
    ), TurretConstants.hoodSimMotor);
    DCMotorSim flywheel = new DCMotorSim(LinearSystemId.createDCMotorSystem(
        TurretConstants.flywheelSimMotor, TurretConstants.flywheelMotorInertiaKgM2, 1
    ), TurretConstants.flywheelSimMotor);

    public TurretSimDCMotor() {
    }

    public void reset() {
        flywheel.setAngle(0);
        flywheel.setAngularVelocity(0);
        hood.setAngle(0);
        hood.setAngularVelocity(0);
        azimuth.setAngle(0);
        azimuth.setAngularVelocity(0);
    }

    public TurretState getState() {
        return new TurretState(
            flywheel.getAngularVelocityRadPerSec(),
            hood.getAngularPositionRad(),
            hood.getAngularVelocityRadPerSec(),
            azimuth.getAngularPositionRad(),
            azimuth.getAngularVelocityRadPerSec()
        );
    }

    private static void applyCurrentBasedDeacceleration(DCMotorSim motorSim, DCMotor motor, double rotorInertia, double current, double dtSeconds) {
        double angVel = motorSim.getAngularVelocityRadPerSec();
        if(angVel < 1e-6) return;

        double torque = motor.getTorque(MathUtil.clamp(current, -80, 80));
        double deacceleration = torque / rotorInertia;
        
        double newAngVel = angVel - deacceleration * dtSeconds;
        motorSim.setAngularVelocity(newAngVel);
    }

    /**
     * Iterate the turret simulation by the given time step, and return the current state.
     */
    public TurretState updateAndGetState(double flywheelVoltage, double hoodVoltage, double azimuthVoltage, double dtSeconds) {
        flywheel.setInputVoltage(flywheelVoltage);
        hood.setInputVoltage(hoodVoltage);
        azimuth.setInputVoltage(azimuthVoltage);

        flywheel.update(dtSeconds);
        hood.update(dtSeconds);
        azimuth.update(dtSeconds);

        // Slow down by the losses from current
        double flywheel_vel = flywheel.getAngularVelocityRadPerSec();
        double hood_vel = hood.getAngularVelocityRadPerSec();
        double azi_vel = azimuth.getAngularVelocityRadPerSec();
        double flywheelSteadyStateCurrent = TurretTuningData.FlywheelCurrentModel.calculate(flywheel_vel, azi_vel, hood_vel);
        double hoodSteadyStateCurrent = TurretTuningData.HoodCurrentModel.calculate(flywheel_vel, azi_vel, hood_vel);
        double azimuthSteadyStateCurrent = TurretTuningData.AzimuthCurrentModel.calculate(flywheel_vel, azi_vel, hood_vel);
        
        applyCurrentBasedDeacceleration(flywheel, TurretConstants.flywheelSimMotor, TurretConstants.flywheelMotorInertiaKgM2, flywheelSteadyStateCurrent, dtSeconds);
        applyCurrentBasedDeacceleration(hood, TurretConstants.hoodSimMotor, TurretConstants.hoodMotorInertiaKgM2, hoodSteadyStateCurrent, dtSeconds);
        applyCurrentBasedDeacceleration(azimuth, TurretConstants.azimuthSimMotor, TurretConstants.azimuthMotorInertiaKgM2, azimuthSteadyStateCurrent, dtSeconds);

        return getState();
    }
}
