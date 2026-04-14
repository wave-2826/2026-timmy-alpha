package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.turret.TurretSim.SimTurretState;

public class TurretIOSim extends TurretIOTalonHighFrequency {
    // Spark simulation objects
    protected TalonFXSimState flywheel1TalonSim = io.flywheel1Talon.getSimState();
    protected TalonFXSimState flywheel2TalonSim = io.flywheel2Talon.getSimState();
    protected TalonFXSimState azimuthTalonSim = io.azimuthTalon.getSimState();
    protected TalonFXSimState hoodTalonSim = io.hoodTalon.getSimState();

    protected TurretSim turretSim = new TurretSim();
    protected SimTurretState turretState = new SimTurretState(0, 0, 0, 0, 0);

    public TurretIOSim() {
        super();

        flywheel1TalonSim.setMotorType(MotorType.KrakenX60);
        flywheel2TalonSim.setMotorType(MotorType.KrakenX60);
        azimuthTalonSim.setMotorType(MotorType.KrakenX60);
        hoodTalonSim.setMotorType(MotorType.KrakenX60);

        flywheel1TalonSim.Orientation = ChassisReference.CounterClockwise_Positive;
        flywheel2TalonSim.Orientation = ChassisReference.CounterClockwise_Positive;

        azimuthTalonSim.Orientation = ChassisReference.CounterClockwise_Positive;
        hoodTalonSim.Orientation = ChassisReference.CounterClockwise_Positive;

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(turretSim::reset));
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

        flywheel1TalonSim.setRotorVelocity(turretState.flywheelMotorVelRps() / (2 * Math.PI));
        flywheel2TalonSim.setRotorVelocity(turretState.flywheelMotorVelRps() / (2 * Math.PI));

        hoodTalonSim.setRawRotorPosition(turretState.hoodMotorPosRad() / (2 * Math.PI));
        hoodTalonSim.setRotorVelocity(turretState.hoodMotorVelRps() / (2 * Math.PI));
        azimuthTalonSim.setRawRotorPosition(turretState.azimuthMotorPosRad() / (2 * Math.PI));
        azimuthTalonSim.setRotorVelocity(turretState.azimuthMotorVelRps() / (2 * Math.PI));

        turretState = turretSim.updateAndGetState(
            flywheel1TalonSim.getTorqueCurrent(),
            -hoodTalonSim.getTorqueCurrent(),
            -azimuthTalonSim.getTorqueCurrent(),
            1. / frequencyHz
        );
    }
}
