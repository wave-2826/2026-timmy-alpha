package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.drive.DriveConstants.SwerveModuleConfig;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation simulation;

    @SuppressWarnings("unchecked")
    public ModuleIOTalonFXSim(SwerveModuleConfig config, SwerveModuleSimulation simulation) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(config.constants()));

        this.simulation = simulation;
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));
        simulation.useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

        inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians)).toArray();

        inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
    }
}
