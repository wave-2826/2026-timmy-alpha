package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.turret.TurretSim.SimTurretState;

public class TurretIOSim extends TurretIOTalonHighFrequency {
    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getKrakenX60Foc(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getKrakenX60Foc(1);
    protected static DCMotor hoodSimMotor = DCMotor.getKrakenX60Foc(1);

    // Spark simulation objects
    protected TalonFXSimState flywheelMotorSim = io.topFlywheelTalon.getSimState();
    protected TalonFXSimState azimuthMotorSim = io.azimuthTalon.getSimState();
    protected TalonFXSimState hoodMotorSim = io.hoodTalon.getSimState();

    protected TurretSim turretSim = new TurretSim();
    protected SimTurretState turretState = new SimTurretState(0, 0, 0, 0, 0);

    public TurretIOSim() {
        super();

        flywheelMotorSim.setMotorType(MotorType.KrakenX60);
        azimuthMotorSim.setMotorType(MotorType.KrakenX60);
        hoodMotorSim.setMotorType(MotorType.KrakenX60);        
    }
  
    @Override
    public synchronized void updateInputs(TurretIOInputs inputs) {
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

    @Override
    protected synchronized void periodic() {
        super.periodic();

        flywheelMotorSim.setRotorVelocity(turretState.flywheelMotorVelRps() / (2 * Math.PI));
        hoodMotorSim.setRawRotorPosition(turretState.hoodMotorPosRad() / (2 * Math.PI));
        hoodMotorSim.setRotorVelocity(turretState.hoodMotorVelRps() / (2 * Math.PI));
        azimuthMotorSim.setRawRotorPosition(turretState.azimuthMotorPosRad() / (2 * Math.PI));
        azimuthMotorSim.setRotorVelocity(turretState.azimuthMotorVelRps() / (2 * Math.PI));

        turretState = turretSim.updateAndGetState(
            flywheelMotorSim.getMotorVoltage(),
            hoodMotorSim.getMotorVoltage(),
            azimuthMotorSim.getMotorVoltage(),
            1. / frequencyHz
        );
    }
}
