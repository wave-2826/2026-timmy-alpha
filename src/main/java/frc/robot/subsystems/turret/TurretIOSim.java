package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.turret.TurretSim.SimTurretState;

public class TurretIOSim extends TurretIOTalonFX {
    private static int subticks = 5;

    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getKrakenX60Foc(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getKrakenX60Foc(1);
    protected static DCMotor hoodSimMotor = DCMotor.getKrakenX60Foc(1);

    protected TalonFXSimState flywheelMotorSim = topFlywheelTalon.getSimState();
    protected TalonFXSimState hoodMotorSim = hoodTalon.getSimState();
    protected TalonFXSimState azimuthMotorSim = azimuthTalon.getSimState();

    protected TurretSim turretSim = new TurretSim();

    public TurretIOSim() {
        super();
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        SimTurretState turretState = new SimTurretState(0, 0, 0, 0, 0);
        for(int i = 0; i < subticks; i++) {
            turretState = turretSim.updateAndGetState(
                flywheelMotorSim.getMotorVoltage(),
                hoodMotorSim.getMotorVoltage(),
                azimuthMotorSim.getMotorVoltage(),
                0.02 / subticks
            );
        }
        
        // Not needed in real life, but needed here because of sim controller error accumulating
        hoodMotorSim.setRawRotorPosition(turretState.hoodMotorPosRad());
        azimuthMotorSim.setRawRotorPosition(turretState.azimuthMotorPosRad());

        azimuthMotorSim.setRotorVelocity(turretSim.getState().azimuthVelRps());
        azimuthMotorSim.setRawRotorPosition(turretSim.getState().azimuthPosRad());

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
}
