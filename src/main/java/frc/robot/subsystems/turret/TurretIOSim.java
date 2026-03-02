package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.turret.TurretSim.TurretSimMode;

public class TurretIOSim extends TurretIOReal {
    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    protected static DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    // Spark simulation objects
    protected SparkSim flywheelMotorSim = new SparkFlexSim(topFlywheelMotor, flywheelSimMotor);
    protected SparkSim azimuthMotorSim = new SparkFlexSim(azimuthMotor, azimuthSimMotor);
    protected SparkSim hoodMotorSim = new SparkFlexSim(hoodMotor, hoodSimMotor);

    // Spark simulation sensors
    protected SparkAbsoluteEncoderSim azimuthEncoderSim = azimuthMotorSim.getAbsoluteEncoderSim();

    protected TurretSim turretSim = new TurretSim(TurretSimMode.LinearSystem);

    public TurretIOSim() {
        super();
    }

    private double calculateTorque(SparkSim motorSim, DCMotor simMotor) {
        double voltage = motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
        // V * Kt / R gives us torque disregarding bEMF, which is already accounted for in the simulation's velocity update
        return voltage * simMotor.KtNMPerAmp / simMotor.rOhms;
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        // TODO: update sim thingies
        // TODO: subtick on main subtick loop
        int subticks = 5;
        for(int i = 0; i < subticks; i++) {
            var turretState = turretSim.updateAndGetState(
                calculateTorque(flywheelMotorSim, flywheelSimMotor),
                calculateTorque(hoodMotorSim, hoodSimMotor),
                calculateTorque(azimuthMotorSim, azimuthSimMotor),
                0.02
            );

            flywheelMotorSim.iterate(turretState.getFlywheelMotorVelocity(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
            hoodMotorSim.iterate(turretState.getHoodMotorVelocity(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
            azimuthMotorSim.iterate(turretState.getAzimuthMotorVelocity(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
        }

        var state = turretSim.getState();
        Logger.recordOutput("TurretSim/State", state);
        Logger.recordOutput("TurretSim/Motors/FlywheelVel", state.getFlywheelMotorVelocity(), RadiansPerSecond);
        Logger.recordOutput("TurretSim/Motors/HoodVel", state.getHoodMotorVelocity(), RadiansPerSecond);
        Logger.recordOutput("TurretSim/Motors/AzimuthVel", state.getAzimuthMotorVelocity(), RadiansPerSecond);

        Logger.recordOutput("TurretSim/FlyTorque", calculateTorque(flywheelMotorSim, flywheelSimMotor));
        Logger.recordOutput("TurretSim/HoodTorque", calculateTorque(hoodMotorSim, hoodSimMotor));
        Logger.recordOutput("TurretSim/AzimuthTorque", calculateTorque(azimuthMotorSim, azimuthSimMotor));

        azimuthEncoderSim.setVelocity(turretSim.getState().azimuthVelocityRps() * 2 * Math.PI);
        azimuthEncoderSim.setPosition(turretSim.getState().azimuthPositionRotations() * 2 * Math.PI);

        super.updateInputs(inputs);

        // Inputs should already bet set, but the top/bottom flywheels need to balance the top sim's
        // since we model them as one motor.
        var distributedFlywheel = inputs.topFlywheel.half();
        inputs.topFlywheel = distributedFlywheel;
        inputs.bottomFlywheel = distributedFlywheel;
    }
}
