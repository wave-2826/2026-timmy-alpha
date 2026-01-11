package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;


public class IntakeIOSpark implements IntakeIO {
    private final SparkMax Intake = new SparkMax(intakeCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = Intake.getEncoder();
  
    public IntakeIOSpark() {
      var config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
      config
          .encoder
          .positionConversionFactor(
              2.0 * Math.PI / motorReduction) // Rotor Rotations -> Intake Radians
          .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
          .uvwMeasurementPeriod(10)
          .uvwAverageDepth(2);
  
      tryUntilOk(
          Intake,
          5,
          () ->
              Intake.configure(
                  config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      ifOk(Intake, encoder::getPosition, (value) -> inputs.positionRad = value);
      ifOk(Intake, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
      ifOk(
          Intake,
          new DoubleSupplier[] {Intake::getAppliedOutput, Intake::getBusVoltage},
          (values) -> inputs.appliedVolts = values[0] * values[1]);
      ifOk(Intake, Intake::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }
  
    @Override
    public void setVoltage(double volts) {
      Intake.setVoltage(volts);
    }
  }
