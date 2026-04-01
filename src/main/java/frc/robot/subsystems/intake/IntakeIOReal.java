package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {
    protected final SparkFlex rollerL = new SparkFlex(intakeRollerLCanId, MotorType.kBrushless);
    protected final SparkFlex rollerR = new SparkFlex(intakeRollerRCanId, MotorType.kBrushless);
    
    protected final SparkMax deployL = new SparkMax(intakeDeployLCanId, MotorType.kBrushless);
    protected final SparkMax deployR = new SparkMax(intakeDeployRCanId, MotorType.kBrushless);
    
    protected final SparkClosedLoopController rollerController = rollerL.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerL = deployL.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerR = deployR.getClosedLoopController();

    protected final RelativeEncoder rollerLEncoder = rollerL.getEncoder();
    protected final RelativeEncoder rollerREncoder = rollerR.getEncoder();
    protected final RelativeEncoder deployEncoderL = deployL.getEncoder();
    protected final RelativeEncoder deployEncoderR = deployR.getEncoder();

    protected final SparkClosedLoopController deployController;

    protected boolean deployFollowing = true;

    public IntakeIOReal() {
        var rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(rollerCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        rollerConfig
            .encoder
            .positionConversionFactor(
                2.0 * Math.PI / rollerMotorReduction) // Rotor Rotations -> Intake Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / rollerMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        rollerConfig.signals.apply(SparkUtil.defaultSignals).primaryEncoderVelocityPeriodMs(20);
        rollerConfig.inverted(true);
        
        var deployBaseConfig = new SparkMaxConfig();
        deployBaseConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(deployCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        deployBaseConfig.encoder
            .positionConversionFactor(2.0 * Math.PI * pinionRadiusMeters / pinionReduction) // Rotor Rotations -> Deploy Meters
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 * pinionRadiusMeters / pinionReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        deployBaseConfig.closedLoop.maxMotion
            // TODO: reasonable values
            .cruiseVelocity(0.8) // m/s
            .maxAcceleration(3.0); // m/s^2
        deployBaseConfig.signals.apply(SparkUtil.defaultSignals).primaryEncoderPositionPeriodMs(20);
        var deployRConfig = new SparkMaxConfig().apply(deployBaseConfig);
        var deployLConfig = new SparkMaxConfig().apply(deployBaseConfig);
        
        deployLConfig.inverted(false);
        deployRConfig.follow(deployL, true);

        var rollerLConfig = new SparkMaxConfig().apply(rollerConfig);
        var rollerRConfig = new SparkMaxConfig().apply(rollerConfig);
        rollerRConfig.follow(rollerL, true);

        IntakeConstants.rollerPID.applyConfigAndRegister(rollerLConfig, rollerL);
        IntakeConstants.rollerPID.applyConfigAndRegister(rollerRConfig, rollerR);
        IntakeConstants.deployPID.applyConfigAndRegister(deployLConfig, deployL);
        IntakeConstants.deployPID.applyConfigAndRegister(deployRConfig, deployR);

        tryUntilOk(rollerL, 5, () -> rollerL.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(rollerR, 5, () -> rollerR.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(deployL, 5, () -> deployL.configure(deployLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(deployR, 5, () -> deployR.configure(deployRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        deployController = deployL.getClosedLoopController();
    }
  
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var rollerLVelocity = getIfOk(rollerL, rollerLEncoder::getVelocity, 0.0);
        var rollerLCurrent = getIfOk(rollerL, rollerL::getOutputCurrent, 0.0);
        inputs.rollerL = new IntakeIOInputs.RollerMotorInputs(!checkFault(), rollerLVelocity, rollerLCurrent);
        
        var rollerRVelocity = getIfOk(rollerR, rollerREncoder::getVelocity, 0.0);
        var rollerRCurrent = getIfOk(rollerR, rollerR::getOutputCurrent, 0.0);
        inputs.rollerR = new IntakeIOInputs.RollerMotorInputs(!checkFault(), rollerRVelocity, rollerRCurrent);

        var deployLCurrent = getIfOk(deployL, deployL::getOutputCurrent, 0.0);
        var deployLPosition = getIfOk(deployL, deployEncoderL::getPosition, 0.0);
        inputs.deployL = new IntakeIOInputs.DeployMotorInputs(!checkFault(), deployLCurrent, deployLPosition);

        var deployRCurrent = getIfOk(deployR, deployR::getOutputCurrent, 0.0);
        var deployRPosition = getIfOk(deployR, deployEncoderR::getPosition, 0.0);
        inputs.deployR = new IntakeIOInputs.DeployMotorInputs(!checkFault(), deployRCurrent, deployRPosition);
    }
  
    @Override
    public void setRollerSpeed(double velocityRPM) {
        rollerController.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM), ControlType.kVelocity);
    }

    @Override
    public void setDeployPowerL(double power) {
        deployL.getClosedLoopController().setSetpoint(power, ControlType.kDutyCycle);
    }

    @Override
    public void setDeployPowerR(double power) {
        if(deployFollowing) {
            deployR.pauseFollowerModeAsync();
            deployFollowing = false;
        }
        deployR.getClosedLoopController().setSetpoint(power, ControlType.kDutyCycle);
    }

    @Override
    public void resetDeployEncoders() {
        deployEncoderL.setPosition(0);
        deployEncoderR.setPosition(0);
    }

    @Override
    public void setDeployPosition(double positionMeters) {
        if(!deployFollowing) {
            deployR.resumeFollowerModeAsync();
            deployFollowing = true;
        }
        deployController.setSetpoint(-positionMeters, ControlType.kPosition);
    }

    @Override
    public void stopDeploy() {
        deployL.stopMotor();
        if(!deployFollowing) {
            deployR.stopMotor();
        }
    }
}
