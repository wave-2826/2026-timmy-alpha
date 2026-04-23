package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

import com.revrobotics.spark.config.SparkFlexConfig;


public class SpindexerIOReal implements SpindexerIO {
    private final SparkFlex spinnerMotor = new SparkFlex(spinnerCanId, MotorType.kBrushless);
    private final SparkMax transferMotor = new SparkMax(transferCanId, MotorType.kBrushless);
    private final RelativeEncoder spinEncoder = spinnerMotor.getEncoder();
    private final RelativeEncoder transferEncoder = transferMotor.getEncoder();
  
    private final Debouncer spinConnectedDebounce = new Debouncer(0.5);
    private final Debouncer transferConnectedDebounce = new Debouncer(0.5);

    public SpindexerIOReal() {
        var spinnerConfig = new SparkFlexConfig();
        spinnerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(spinnerCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        spinnerConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / spinnerMotorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / spinnerMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        spinnerConfig.inverted(true);
        
        var tranferConfig = new SparkMaxConfig();
        tranferConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(transferCurrentLimit).voltageCompensation(Constants.voltageCompensation);
        tranferConfig.closedLoopRampRate(0.5);
        tranferConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / transferMotorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / transferMotorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        SpindexerConstants.spinnerPID.applyConfigAndRegister(spinnerConfig, spinnerMotor);
        SpindexerConstants.transferPID.applyConfigAndRegister(tranferConfig, transferMotor);

        tryUntilOk(spinnerMotor, 5, () ->
            spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(transferMotor, 5, () ->
            transferMotor.configure(tranferConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        var spinnerVelocity = getIfOk(spinnerMotor, spinEncoder::getVelocity, 0.0);
        var spinnerCurrent = getIfOk(spinnerMotor, spinnerMotor::getOutputCurrent, 0.0);
        inputs.spinner = new SpindexerIOInputs.SpinnerMotorInputs(spinConnectedDebounce.calculate(!checkFault()), spinnerVelocity, spinnerCurrent);

        var transferVelocity = getIfOk(transferMotor, transferEncoder::getVelocity, 0.0);
        var transferCurrent = getIfOk(transferMotor, transferMotor::getOutputCurrent, 0.0);
        inputs.transfer = new SpindexerIOInputs.TransferMotorInputs(transferConnectedDebounce.calculate(!checkFault()), transferVelocity, transferCurrent);
    }
  
    @Override
    public void setSpinnerPower(double power) {
        spinnerMotor.getClosedLoopController().setSetpoint(power, ControlType.kDutyCycle);
    }

    @Override
    public void setTransferPower(double power) {
        transferMotor.getClosedLoopController().setSetpoint(power, ControlType.kDutyCycle);
    }
}
