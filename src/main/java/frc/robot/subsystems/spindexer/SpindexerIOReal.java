package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
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

public class SpindexerIOReal implements SpindexerIO {
    private final SparkMax spinnerMotor = new SparkMax(spinnerCanId, MotorType.kBrushless);
    private final SparkMax transferMotor = new SparkMax(transferCanId, MotorType.kBrushless);
    private final RelativeEncoder spinEncoder = spinnerMotor.getEncoder();
    private final RelativeEncoder transferEncoder = transferMotor.getEncoder();
  
    public SpindexerIOReal() {
        var spinnerConfig = new SparkMaxConfig();
        spinnerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(spinnerCurrentLimit).voltageCompensation(12.0);
        spinnerConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / spinnerMotorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / spinnerCurrentLimit)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        var tranferConfig = new SparkMaxConfig();
        tranferConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(transferCurrentLimit).voltageCompensation(12.0);
        tranferConfig
            .encoder
            .positionConversionFactor(2.0 * Math.PI / transferMotorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / transferCurrentLimit)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        tryUntilOk(spinnerMotor, 5, () ->
            spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(transferMotor, 5, () ->
            transferMotor.configure(tranferConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        var spinnerVelocity = getIfOk(spinnerMotor, spinEncoder::getVelocity, 0.0);
        var spinnerCurrent = getIfOk(spinnerMotor, spinnerMotor::getOutputCurrent, 0.0);
        inputs.spinner = new SpindexerIOInputs.SpinnerMotorInputs(sparkStickyFault, spinnerVelocity, spinnerCurrent);

        var transferVelocity = getIfOk(transferMotor, transferEncoder::getVelocity, 0.0);
        var transferCurrent = getIfOk(transferMotor, transferMotor::getOutputCurrent, 0.0);
        inputs.spinner = new SpindexerIOInputs.SpinnerMotorInputs(sparkStickyFault, transferVelocity, transferCurrent);
    }
  
    @Override
    public void setSpinnerVoltage(double power) {
        spinnerMotor.set(power);
    }

    @Override
    public void setTransferVoltage(double power) {
        transferMotor.set(power);
    }
}
