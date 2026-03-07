package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.util.simUtils.spark.SparkSimThatActuallyWorks;

public class TurretIOSim extends TurretIOReal {
    private static int subticks = 5;

    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    protected static DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    // Spark simulation objects
    protected SparkSimThatActuallyWorks flywheelMotorSim = new SparkSimThatActuallyWorks(topFlywheelMotor, "top flywheel", flywheelSimMotor);
    protected SparkSimThatActuallyWorks azimuthMotorSim = new SparkSimThatActuallyWorks(azimuthMotor, "azimuth", azimuthSimMotor);
    protected SparkSimThatActuallyWorks hoodMotorSim = new SparkSimThatActuallyWorks(hoodMotor, "hood", hoodSimMotor);

    // Spark simulation sensors
    protected SparkAbsoluteEncoderSim azimuthEncoderSim = azimuthMotorSim.getAbsoluteEncoderSim();

    protected TurretSim turretSim = new TurretSim();

    public TurretIOSim() {
        super();
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        if(!DriverStationSim.getDsAttached()) {
            turretSim.reset();
            return;
        }

        // TODO: subtick on main subtick loop

        for(int i = 0; i < subticks; i++) {
            var turretState = turretSim.updateAndGetState(
                flywheelMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage(),
                hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage(),
                azimuthMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage(),
                0.02 / subticks
            );

            flywheelMotorSim.iterate(turretState.flywheelMotorVelRps(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
            hoodMotorSim.iterate(turretState.hoodMotorVelRps(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
            azimuthMotorSim.iterate(turretState.azimuthMotorVelRps(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
        
            // Not needed in real life, but needed here because of sim controller error accumulating
            hoodMotorSim.setPosition(turretState.hoodMotorPosRad());
            azimuthMotorSim.setPosition(turretState.azimuthMotorPosRad());

            azimuthEncoderSim.setVelocity(turretSim.getState().azimuthVelRps());
            azimuthEncoderSim.setPosition(turretSim.getState().azimuthPosRad());
        }

        Logger.recordOutput("TurretSim/Setpoints/FlySetpoint", flywheelMotorSim.getSetpoint());
        Logger.recordOutput("TurretSim/Setpoints/HoodSetpoint", hoodMotorSim.getSetpoint());
        Logger.recordOutput("TurretSim/Setpoints/AzimuthSetpoint", azimuthMotorSim.getSetpoint());

        var state = turretSim.getState();
        Logger.recordOutput("TurretSim/State", state);
        Logger.recordOutput("TurretSim/State/HoodPosRad", state.hoodPosRad());

        super.updateInputs(inputs);

        // Inputs should already bet set, but the top/bottom flywheels need to balance the top sim's
        // since we model them as one motor.
        var distributedFlywheel = inputs.topFlywheel.half();
        inputs.topFlywheel = distributedFlywheel;
        inputs.bottomFlywheel = distributedFlywheel;
    }

    // private double manualVoltageCompensation(double voltage) {
    //     return voltage * 13.4 / RobotController.getBatteryVoltage();
    // }
    // private double getVoltage(DCMotor motor, double current, double speedRadiansPerSec) {
    //     return manualVoltageCompensation(1.0 / motor.KvRadPerSecPerVolt * speedRadiansPerSec) +
    //         motor.rOhms * current;
    // }
}
