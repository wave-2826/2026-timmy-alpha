package frc.robot.subsystems.drive;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
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

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DriveConstants.driveMotorModel, 0.025, DriveConstants.driveGearRatio),
        DriveConstants.driveMotorModel
    );
    private final DCMotorSim turnSim;
    
    public ModuleIOSim(SwerveModuleConfig config) {
        super(config);

        driveSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
        turnSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

        // Set up turn sim (depends on index for correct reduction)
        turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DriveConstants.turnMotorModel,
            0.004,
            DriveConstants.steerGearRatio
        ), DriveConstants.turnMotorModel);
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        turnSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        var driveMotorVoltage = driveSimState.getMotorVoltage();
        var turnMotorVoltage = turnSimState.getMotorVoltage();

        // Update simulation state
        driveSim.setInputVoltage(driveMotorVoltage);
        turnSim.setInputVoltage(turnMotorVoltage);

        driveSim.update(0.02);
        turnSim.update(0.02);

        driveSimState.setRawRotorPosition(driveSim.getAngularPositionRad() * DriveConstants.driveGearRatio);
        driveSimState.setRotorVelocity(driveSim.getAngularVelocityRadPerSec() * DriveConstants.driveGearRatio);

        turnSimState.setRawRotorPosition(turnSim.getAngularPositionRad() * DriveConstants.steerGearRatio);
        turnSimState.setRotorVelocity(turnSim.getAngularVelocityRadPerSec() * DriveConstants.steerGearRatio);
        
        super.updateInputs(inputs);
    }
}