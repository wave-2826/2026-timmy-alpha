package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.util.LocalADStarAK;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final RobotState robotState = RobotState.getInstance();
    
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, "FrontLeft");
        modules[1] = new Module(frModuleIO, "FrontRight");
        modules[2] = new Module(blModuleIO, "BackLeft");
        modules[3] = new Module(brModuleIO, "BackRight");

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        var robotState = RobotState.getInstance();
        AutoBuilder.configure(robotState::getPose, this::setPose, this::getChassisSpeeds,
            this::runVelocityWithFeedforward,
            Constants.isSim ? DriveConstants.simHolonomicDriveController : DriveConstants.realHolonomicDriveController,
            DriveConstants.pathplannerConfig, () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for(var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if(DriverStation.isDisabled()) {
            for(var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if(DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        var robotState = RobotState.getInstance();
        for(int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }

            robotState.applyOdometryUpdate(sampleTimestamps[i], modulePositions,
                gyroInputs.connected ? Optional.of(gyroInputs.odometryYawPositions[i]) : Optional.empty());
        }
        robotState.addDriveSpeeds(getChassisSpeeds());

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired robot-relative velocity with the specified feedforwards.
     *
     * @param speeds
     */
    public void runVelocityWithFeedforward(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        runVelocity(speeds, feedforwards.accelerationsMPSSq());
    }

    /**
     * Runs the drive at the desired robot-relative velocity. Doesn't account for acceleration.
     * @param speeds
     */
    public void runVelocity(ChassisSpeeds speeds) {
        runVelocity(speeds, new double[4]);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     * @param accelerations Accelerations **assuming unoptimized module states**.
     */
    @SuppressWarnings("unused")
    public void runVelocity(ChassisSpeeds speeds, double[] accelerationsMps2) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = robotState.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.linearFreeSpeed);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for(int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i], accelerationsMps2[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for(int i = 0; i < 4; i++) modules[i].runCharacterization(output);
    }
    /** Runs a particular module in a straight line with the specified drive output. */
    public void runCharacterization(int module, double output) {
        modules[module].runCharacterization(output);
    }

    /** Runs the drive to rotate with the specified drive output for angular system identification. */
    public void runAngularCharacterization(double output) {
        for(int i = 0; i < 4; i++) modules[i].runAngularCharacterization(output);
    }

    /** Sets the current limit on the drive motors temporarily for slip current measurement. */
    public void setSlipMeasurementCurrentLimit(Current limit) {
        for(int i = 0; i < 4; i++) modules[i].setSlipMeasurementCurrentLimit(limit);
    }
    /** Returns the drive motor current draw of a particular module in amps. */
    public double getSlipMeasurementCurrent(int module) {
        return modules[module].getSlipMeasurementCurrent();
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }
    
    /** Returns the drive motor position of a particular module in radians. */
    public double getSlipMeasurementPosition(int module) {
        return modules[module].getWheelRadiusCharacterizationPosition();
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = DriveConstants.moduleTranslations[i].getAngle();
        }
        robotState.kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Resets the odometry to the specified pose. Used at the start of autonomous to tell the robot where it is.
     * @return
     */
    public void setPose(Pose2d pose) {
        RobotState.getInstance().setPose(pose, getModulePositions());
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return robotState.kinematics.toChassisSpeeds(getModuleStates());
    }
}
