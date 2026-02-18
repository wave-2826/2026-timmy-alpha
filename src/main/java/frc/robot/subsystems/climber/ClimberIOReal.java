package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax leftMotor = new SparkMax(leftCanId, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(rightCanId, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  
    public ClimberIOReal() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        config
            .encoder
            .positionConversionFactor(2.0 * Math.PI / motorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        
        ClimberConstants.climbPID.applyConfigAndRegister(config, rightMotor);
        ClimberConstants.climbPID.applyConfigAndRegister(config, leftMotor);

  
        tryUntilOk(rightMotor, 5, () ->
            rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(leftMotor, 5, () ->
            leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        var climbRCurrent = getIfOk(rightMotor, rightMotor::getOutputCurrent, 0.0);
        var climbRPosition = getIfOk(rightMotor, rightEncoder::getPosition, 0.0);
        inputs.right = new ClimberIOInputs.climbMotorInputs(sparkStickyFault, climbRCurrent, climbRPosition);

        var climbLCurrent = getIfOk(leftMotor, leftMotor::getOutputCurrent, 0.0);
        var climbLPosition = getIfOk(leftMotor, leftEncoder::getPosition, 0.0);
        inputs.left = new ClimberIOInputs.climbMotorInputs(sparkStickyFault, climbLCurrent, climbLPosition);
    }
  
    @Override
    public void setLeftPower(double power) {
        leftMotor.set(power);
    }

    @Override
    public void setRightPower(double power) {
        rightMotor.set(power);
    }
}
