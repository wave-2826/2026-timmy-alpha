package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import au.grapplerobotics.LaserCan;
import java.util.function.DoubleSupplier;

public class IntakeIOReal implements IntakeIO {
    protected final SparkMax roller = new SparkMax(intakeRollerCanId, MotorType.kBrushless);
    protected final SparkMax deployL = new SparkMax(intakeDeployLCanId, MotorType.kBrushless);
    protected final SparkMax deployR = new SparkMax(intakeDeployRCanId, MotorType.kBrushless);
    
    protected final SparkClosedLoopController rollerController = roller.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerL = deployL.getClosedLoopController();
    protected final SparkClosedLoopController deployControllerR = deployR.getClosedLoopController();

    protected final RelativeEncoder rollerEncoder = roller.getEncoder();
    protected final LaserCan deployLCan = new LaserCan(intakeDeployLLaserCanId);
    protected final LaserCan deployRCan = new LaserCan(intakeDeployRLaserCanId);

    public IntakeIOReal() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        config
            .encoder
            .positionConversionFactor(
                2.0 * Math.PI / motorReduction) // Rotor Rotations -> Intake Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
    
        IntakeConstants.rollerPID.applyConfigAndRegister(config, roller);

        tryUntilOk(roller, 5, () -> roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        ifOk(roller, rollerEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            roller,
            new DoubleSupplier[] {roller::getAppliedOutput, roller::getBusVoltage},
            (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(roller, roller::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }
  
    @Override
    public void setVoltage(double volts) {
        roller.setVoltage(volts);
    }
}
