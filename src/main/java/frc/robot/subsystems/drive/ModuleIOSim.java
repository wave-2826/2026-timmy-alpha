package frc.robot.subsystems.drive;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.DriveConstants.SwerveModuleConfig;

/**
* Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
* Simulation is always based on voltage control.
*/
public class ModuleIOSim extends ModuleIOTalonFX {
    // TODO: we can simulate more accurately if we run at a higher rate. For some reason, ctre
    // doesn't let us tick the simulation manually... so we'll need to use a notifier

    private final TalonFXSimState driveSimState = driveTalon.getSimState();
    private final TalonFXSimState turnSimState = turnTalon.getSimState();
    private final CANcoderSimState cancoderSimState = cancoder.getSimState();

    private final double driveInertia = 0.02;
    private final double turnInertia = 0.01;

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DriveConstants.driveMotorModel, driveInertia, DriveConstants.driveGearRatio),
        DriveConstants.driveMotorModel
    );
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DriveConstants.turnMotorModel, turnInertia,DriveConstants.steerGearRatio),
        DriveConstants.turnMotorModel
    );
    
    public ModuleIOSim(SwerveModuleConfig config) {
        super(config);

        driveSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        turnSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        turnSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage()); // Not sure why it needs to know but...
        
        var driveMotorVoltage = driveSimState.getMotorVoltage();
        var turnMotorVoltage = turnSimState.getMotorVoltage();

        // Update simulation state
        driveSim.setInputVoltage(driveMotorVoltage);
        turnSim.setInputVoltage(turnMotorVoltage);

        driveSim.update(0.02);
        turnSim.update(0.02);

        // Constant resistant coulomb friction
        double frictionNM = 0.05;
        driveSim.setAngularVelocity(driveSim.getAngularVelocityRadPerSec() + Math.tanh(10. * driveSim.getAngularVelocityRadPerSec()) * frictionNM / driveInertia);
        turnSim.setAngularVelocity(turnSim.getAngularVelocityRadPerSec() + Math.tanh(10. * turnSim.getAngularVelocityRadPerSec()) * frictionNM / turnInertia);

        // Linear drive friction from things like carpet interactions, air resistence, idk what else but probably a bunch
        double driveFrictionNMSperRad = 0.001;
        driveSim.setAngularVelocity(driveSim.getAngularVelocityRadPerSec() - driveSim.getAngularVelocityRadPerSec() * driveFrictionNMSperRad / driveInertia);

        driveSimState.setRawRotorPosition(driveSim.getAngularPositionRad());
        driveSimState.setRotorVelocity(driveSim.getAngularVelocityRadPerSec());

        turnSimState.setRawRotorPosition(turnSim.getAngularPositionRad());
        turnSimState.setRotorVelocity(turnSim.getAngularVelocityRadPerSec());

        cancoderSimState.setRawPosition(turnSim.getAngularPositionRad());
        cancoderSimState.setVelocity(turnSim.getAngularVelocityRadPerSec());
        
        super.updateInputs(inputs);

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnAbsolutePosition};
    }
}