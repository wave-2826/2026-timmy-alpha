package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class TurretIOReal implements TurretIO {
    protected final SparkFlex topFlywheelMotor    = new SparkFlex(topFlywheelCanID, MotorType.kBrushless);
    protected final SparkFlex bottomFlywheelMotor = new SparkFlex(bottomFlywheelCanID, MotorType.kBrushless);
    protected final SparkFlex azimuthMotor        = new SparkFlex(azimuthCanID, MotorType.kBrushless);
    protected final SparkFlex hoodMotor           = new SparkFlex(hoodCanID, MotorType.kBrushless);

    public TurretIOReal() {
        // var config = new SparkMaxConfig();
        // config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        // config.encoder
        //     .positionConversionFactor(2.0 * Math.PI / motorReduction) // Rotor Rotations -> Radians
        //     .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
        //     .uvwMeasurementPeriod(10)
        //     .uvwAverageDepth(2);
        
        // tryUntilOk(motor, 5, () ->
        //     motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // ifOk(motor, encoder::getPosition, (value) -> inputs.positionRad = value);
        // ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        // ifOk(
        //     motor,
        //     new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        //     (values) -> inputs.appliedVolts = values[0] * values[1]);
        // ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }
}
