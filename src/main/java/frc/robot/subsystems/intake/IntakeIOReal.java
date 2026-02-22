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
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {
    protected final SparkFlex roller = new SparkFlex(intakeRollerCanId, MotorType.kBrushless);
    protected final SparkMax deployL = new SparkMax(intakeDeployLCanId, MotorType.kBrushless);
    protected final SparkMax deployR = new SparkMax(intakeDeployRCanId, MotorType.kBrushless);
    
    protected final SparkClosedLoopController rollerController = roller.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerL = deployL.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerR = deployR.getClosedLoopController();

    protected final RelativeEncoder rollerEncoder = roller.getEncoder();
    protected final RelativeEncoder deployEncoderL = deployL.getEncoder();
    protected final RelativeEncoder deployEncoderR = deployR.getEncoder();

    protected final SparkClosedLoopController deployController;

    public IntakeIOReal() {
        var rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(rollerCurrentLimit).voltageCompensation(12.0);
        rollerConfig
            .encoder
            .positionConversionFactor(
                2.0 * Math.PI / rollerMotorReduction) // Rotor Rotations -> Intake Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / rollerMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        
        var deployBaseConfig = new SparkMaxConfig();
        deployBaseConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(deployCurrentLimit).voltageCompensation(12.0);
        deployBaseConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI * pinionRadiusMeters)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 * pinionRadiusMeters)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        var deployRConfig = new SparkMaxConfig().apply(deployBaseConfig);
        var deployLConfig = new SparkMaxConfig().apply(deployBaseConfig);
        deployRConfig.follow(deployL, true);

        IntakeConstants.rollerPID.applyConfigAndRegister(rollerConfig, roller);
        IntakeConstants.deployPID.applyConfigAndRegister(deployLConfig, deployL);
        IntakeConstants.deployPID.applyConfigAndRegister(deployRConfig, deployR);

        tryUntilOk(roller, 5, () -> roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(deployL, 5, () -> deployL.configure(deployLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(deployR, 5, () -> deployR.configure(deployRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        deployController = deployL.getClosedLoopController();
    }
  
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var rollerVelocity = getIfOk(roller, rollerEncoder::getVelocity, 0.0);
        var rollerCurrent = getIfOk(roller, roller::getOutputCurrent, 0.0);
        inputs.roller = new IntakeIOInputs.RollerMotorInputs(sparkStickyFault, rollerVelocity, rollerCurrent);

        var deployLCurrent = getIfOk(deployL, deployL::getOutputCurrent, 0.0);
        var deployLPosition = getIfOk(deployL, deployEncoderL::getPosition, 0.0);
        inputs.deployL = new IntakeIOInputs.DeployMotorInputs(sparkStickyFault, deployLCurrent, deployLPosition);

        var deployRCurrent = getIfOk(deployR, deployR::getOutputCurrent, 0.0);
        var deployRPosition = getIfOk(deployR, deployEncoderR::getPosition, 0.0);
        inputs.deployR = new IntakeIOInputs.DeployMotorInputs(sparkStickyFault, deployRCurrent, deployRPosition);
    }
  
    @Override
    public void setRollerVoltage(double volts) {
        roller.setVoltage(volts);
    }

    @Override
    public void setDeployVoltage(double volts) {
        roller.setVoltage(volts);
    }

    @Override
    public void resetDeployEncoders() {
        deployEncoderL.setPosition(0);
        deployEncoderR.setPosition(0);
    }

    @Override
    public void setDeployPosition(double position) {
        deployController.setSetpoint(position, ControlType.kPosition);
    }
}
