package frc.robot.subsystems.$name;

import static frc.robot.subsystems.$name.$NameConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class $NameIOReal implements $NameIO {
    private final SparkMax motor = new SparkMax(motorCanId, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
  
    public $NameIOReal() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        config
            .encoder
            .positionConversionFactor(2.0 * Math.PI / motorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
  
        tryUntilOk(motor, 5, () ->
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs($NameIOInputs inputs) {
        ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            motor,
            new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
            (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }
  
    @Override
    public void setPower(double power) {
        motor.set(power);
    }
}
