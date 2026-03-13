package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
* Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
* Simulation is always based on voltage control.
*/
public class ModuleIOSim implements ModuleIO {
    private static final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor turnMotorModel = DCMotor.getKrakenX60Foc(1);
    
    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, DriveConstants.driveGearRatio),
        driveMotorModel
    );
    private final DCMotorSim turnSim;
    
    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController driveController = new PIDController(0, 0, 0, 0.02);
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    private PIDController turnController = new PIDController(0, 0, 0, 0.02);
    
    private double driveFFVolts = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    
    public ModuleIOSim(int index) {
        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        // DriveConstants.
        
        // Set up turn sim (depends on index for correct reduction)
        turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            turnMotorModel,
            0.004,
            DriveConstants.steerGearRatio
        ), turnMotorModel);
    }
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if(driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if(turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }
        
        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);
        
        inputs.driveConnected = true;
        inputs.drivePositionRad = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        
        inputs.turnConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    }
    
    @Override
    public void setDriveOpenLoopCurrent(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoopCurrent(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double accelerationRadPerSec2) {
        driveClosedLoop = true;
        driveFFVolts = driveFeedforward.calculateWithVelocities(velocityRadPerSec, velocityRadPerSec + 0.02 * accelerationRadPerSec2);
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {
        driveController.setPID(kP, kI, kD);
        driveFeedforward.setKs(kS);
        driveFeedforward.setKv(kV);
        driveFeedforward.setKa(kA);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnController.setPID(kP, kI, kD);
    }
}