package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.turret.TurretSim.SimTurretState;

public class TurretIOSim extends TurretIOTalonHighFrequency {
    // Spark simulation objects
    protected TalonFXSimState flywheelTalonSim = io.topFlywheelTalon.getSimState();
    protected TalonFXSimState bottomFlywheelTalonSim = io.bottomFlywheelTalon.getSimState();
    protected TalonFXSimState azimuthTalonSim = io.azimuthTalon.getSimState();
    protected TalonFXSimState hoodTalonSim = io.hoodTalon.getSimState();

    protected TurretSim turretSim = new TurretSim();
    protected SimTurretState turretState = new SimTurretState(0, 0, 0, 0, 0);

    public TurretIOSim() {
        super();

        flywheelTalonSim.setMotorType(MotorType.KrakenX60);
        bottomFlywheelTalonSim.setMotorType(MotorType.KrakenX60);
        azimuthTalonSim.setMotorType(MotorType.KrakenX60);
        hoodTalonSim.setMotorType(MotorType.KrakenX60);

        flywheelTalonSim.Orientation = ChassisReference.CounterClockwise_Positive;
        bottomFlywheelTalonSim.Orientation = ChassisReference.Clockwise_Positive;

        azimuthTalonSim.Orientation = ChassisReference.CounterClockwise_Positive;
        hoodTalonSim.Orientation = ChassisReference.CounterClockwise_Positive;
    }
  
    @Override
    public synchronized void updateInputs(TurretIOInputs inputs) {
        var state = turretSim.getState();
        Logger.recordOutput("TurretSim/State", state);
        Logger.recordOutput("TurretSim/State/HoodPosRad", state.hoodPosRad());

        super.updateInputs(inputs);

        inputs.azimuthZeroTriggered = Math.abs(state.azimuthPosRad() - TurretConstants.azimuthResetAngle.getRadians()) < Units.degreesToRadians(2);
    }

    @Override
    protected synchronized void periodic() {
        super.periodic();

        flywheelTalonSim.setRotorVelocity(turretState.flywheelMotorVelRps() / (2 * Math.PI));
        bottomFlywheelTalonSim.setRotorVelocity(turretState.flywheelMotorVelRps() / (2 * Math.PI));

        hoodTalonSim.setRawRotorPosition(turretState.hoodMotorPosRad() / (2 * Math.PI));
        hoodTalonSim.setRotorVelocity(turretState.hoodMotorVelRps() / (2 * Math.PI));
        azimuthTalonSim.setRawRotorPosition(turretState.azimuthMotorPosRad() / (2 * Math.PI));
        azimuthTalonSim.setRotorVelocity(turretState.azimuthMotorVelRps() / (2 * Math.PI));

        turretState = turretSim.updateAndGetState(
            -flywheelTalonSim.getTorqueCurrent(),
            -hoodTalonSim.getTorqueCurrent(),
            -azimuthTalonSim.getTorqueCurrent(),
            1. / frequencyHz
        );
    }
}
