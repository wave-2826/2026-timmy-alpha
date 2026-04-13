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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {
    protected final SparkFlex rollerL = new SparkFlex(intakeRollerLCanId, MotorType.kBrushless);
    protected final SparkFlex rollerR = new SparkFlex(intakeRollerRCanId, MotorType.kBrushless);
    
    protected final SparkMax deployL = new SparkMax(intakeDeployLCanId, MotorType.kBrushless);
    protected final SparkMax deployR = new SparkMax(intakeDeployRCanId, MotorType.kBrushless);
    
    protected final SparkClosedLoopController rollerLController = rollerL.getClosedLoopController();
    protected final SparkClosedLoopController rollerRController = rollerR.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerL = deployL.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerR = deployR.getClosedLoopController();

    protected final RelativeEncoder rollerLEncoder = rollerL.getEncoder();
    protected final RelativeEncoder rollerREncoder = rollerR.getEncoder();
    protected final RelativeEncoder deployEncoderL = deployL.getEncoder();
    protected final RelativeEncoder deployEncoderR = deployR.getEncoder();

    protected final Debouncer deployLConnectedDebouncer = new Debouncer(0.5);
    protected final Debouncer deployRConnectedDebouncer = new Debouncer(0.5);
    protected final Debouncer rollerLConnectedDebouncer = new Debouncer(0.5);
    protected final Debouncer rollerRConnectedDebouncer = new Debouncer(0.5);

    protected final SparkClosedLoopController deployLController;
    protected final SparkClosedLoopController deployRController;

    public IntakeIOReal() {
        // General configs
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
        
        var deployBaseConfig = new SparkMaxConfig();
        deployBaseConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(deployCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        deployBaseConfig.inverted(true);
        deployBaseConfig.encoder
            .positionConversionFactor(2.0 * Math.PI * pinionRadiusMeters / pinionReduction) // Rotor Rotations -> Deploy Meters
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 * pinionRadiusMeters / pinionReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        deployBaseConfig.closedLoop.maxMotion
            // TODO: reasonable values
            .cruiseVelocity(2.0) // m/s
            .maxAcceleration(3.0); // m/s^2
        deployBaseConfig.signals.apply(SparkUtil.defaultSignals).primaryEncoderPositionPeriodMs(20);

        // Per-motor
        var deployRConfig = new SparkMaxConfig().apply(deployBaseConfig);
        var deployLConfig = new SparkMaxConfig().apply(deployBaseConfig);
        
        var rollerLConfig = new SparkMaxConfig().apply(rollerConfig);
        var rollerRConfig = new SparkMaxConfig().apply(rollerConfig);
        rollerLConfig.inverted(false);
        rollerRConfig.inverted(true);

        IntakeConstants.rollerPID.applyConfigAndRegister(rollerLConfig, rollerL);
        IntakeConstants.rollerPID.applyConfigAndRegister(rollerRConfig, rollerR);
        IntakeConstants.deployPID.applyConfigAndRegister(deployLConfig, deployL);
        IntakeConstants.deployPID.applyConfigAndRegister(deployRConfig, deployR);

        tryUntilOk(rollerL, 5, () -> rollerL.configure(rollerLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(rollerR, 5, () -> rollerR.configure(rollerRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(deployL, 5, () -> deployL.configure(deployLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(deployR, 5, () -> deployR.configure(deployRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        deployLController = deployL.getClosedLoopController();
        deployRController = deployR.getClosedLoopController();
    }
  
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var rollerLVelocity = getIfOk(rollerL, rollerLEncoder::getVelocity, 0.0);
        var rollerLCurrent = getIfOk(rollerL, rollerL::getOutputCurrent, 0.0);
        inputs.rollerL = new IntakeIOInputs.RollerMotorInputs(
            rollerLConnectedDebouncer.calculate(!checkFault()),
            rollerLVelocity, rollerLCurrent
        );
        
        var rollerRVelocity = getIfOk(rollerR, () -> -rollerREncoder.getVelocity(), 0.0);
        var rollerRCurrent = getIfOk(rollerR, rollerR::getOutputCurrent, 0.0);
        inputs.rollerR = new IntakeIOInputs.RollerMotorInputs(
            rollerRConnectedDebouncer.calculate(!checkFault()),
            rollerRVelocity, rollerRCurrent
        );

        var deployLCurrent = getIfOk(deployL, deployL::getOutputCurrent, 0.0);
        var deployLPosition = getIfOk(deployL, deployEncoderL::getPosition, 0.0);
        inputs.deployL = new IntakeIOInputs.DeployMotorInputs(
            deployLConnectedDebouncer.calculate(!checkFault()),
            deployLCurrent, deployLPosition
        );

        var deployRCurrent = getIfOk(deployR, deployR::getOutputCurrent, 0.0);
        var deployRPosition = getIfOk(deployR, deployEncoderR::getPosition, 0.0);
        inputs.deployR = new IntakeIOInputs.DeployMotorInputs(
            deployRConnectedDebouncer.calculate(!checkFault()),
            deployRCurrent, deployRPosition
        );
    }
  
    @Override
    public void setRollerSpeed(double velocityRPM) {
        rollerLController.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM), ControlType.kVelocity);
        rollerRController.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM), ControlType.kVelocity);
    }

    @Override
    public void setDeployPowerL(double power) {
        deployL.getClosedLoopController().setSetpoint(power, ControlType.kDutyCycle);
    }

    @Override
    public void setDeployPowerR(double power) {
        deployR.getClosedLoopController().setSetpoint(-power, ControlType.kDutyCycle);
    }

    @Override
    public void resetDeployEncoders() {
        deployEncoderL.setPosition(0);
        deployEncoderR.setPosition(0);
    }

    @Override
    public void setDeployPosition(double positionMeters) {
        deployRController.setSetpoint(positionMeters, ControlType.kPosition);
        deployLController.setSetpoint(-positionMeters, ControlType.kPosition);
    }

    @Override
    public void stopDeploy() {
        deployL.stopMotor();
        deployR.stopMotor();
    }
}
