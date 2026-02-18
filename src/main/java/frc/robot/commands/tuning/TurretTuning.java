package frc.robot.commands.tuning;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.util.SparkPIDConstants;
import frc.robot.util.SparkUtil;
import frc.robot.util.tunables.TunableSparkPID;

import static frc.robot.util.SparkUtil.tryUntilOk;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class does NOT work for replay! It's only for testing and tuning purposes.
 */
public class TurretTuning {
    private int activeMotor = 0;
    private boolean running = false;
    private boolean initialized = false;

    private SparkClosedLoopController bottomFlywheelController;

    private TunableSparkPID tunablePID = new TunableSparkPID("TurretTuning/FlywheelPID")
        .addRealRobotGains(new SparkPIDConstants(0.0002, 0.0, 0.0, 0.0019));

    public void start(TurretIOReal io, CommandXboxController controller) {
        if(running) return;
        running = true;

        var testMotorConfig = new SparkFlexConfig();
        testMotorConfig.signals.apply(SparkUtil.defaultSignals);
        testMotorConfig.signals.outputCurrentPeriodMs(20).primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20);
        tunablePID.applyConfigAndRegister(testMotorConfig, io.topFlywheelMotor, io.bottomFlywheelMotor, io.azimuthMotor, io.hoodMotor);
        testMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(Constants.voltageCompensation);
        testMotorConfig.encoder
            .positionConversionFactor(2.0 * Math.PI) // Rotor Rotations -> Radians (of ring)
            .velocityConversionFactor((2.0 * Math.PI) / 60.0); // RPM -> rad/s (of ring)
        
        tryUntilOk(io.topFlywheelMotor, 5, () -> io.topFlywheelMotor.configure(testMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(io.bottomFlywheelMotor, 5, () -> io.bottomFlywheelMotor.configure(testMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(io.azimuthMotor, 5, () -> io.azimuthMotor.configure(testMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(io.hoodMotor, 5, () -> io.hoodMotor.configure(testMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        bottomFlywheelController = io.bottomFlywheelMotor.getClosedLoopController();

        initialize(controller);
    }

    private void initialize(CommandXboxController controller) {
        if(initialized) return;
        initialized = true;
        
        controller.rightBumper().onTrue(Commands.runOnce(this::nextMotor));
        controller.leftBumper().onTrue(Commands.runOnce(this::previousMotor));
        controller.a().onTrue(Commands.runOnce(this::resetAverage));
    }

    private void nextMotor() {
        activeMotor = (activeMotor + 1) % 4;
    }
    private void previousMotor() {
        activeMotor = (activeMotor - 1 + 4) % 4;
    }

    private void resetAverage() {
        averageSamples = 0;
        for(int i = 0; i < averageSums.length; i++) {
            averageSums[i] = 0.0;
        }
    }

    private LoggedNetworkNumber[] motorSpeeds = {
        new LoggedNetworkNumber("Tuning/TurretTuning/TopFlywheelPower", 0.0),
        new LoggedNetworkNumber("Tuning/TurretTuning/BottomFlywheelPower", 0.0),
        new LoggedNetworkNumber("Tuning/TurretTuning/AzimuthPower", 0.0),
        new LoggedNetworkNumber("Tuning/TurretTuning/HoodPower", 0.0)
    };

    private int averageSamples = 0;
    private double[] averageSums = new double[4];

    private record Motor(String name, SparkClosedLoopController controller, SparkBase motor) {}

    public void run(TurretIOReal io, CommandXboxController controller) {
        if(!running) return;

        double changePerSecond = 100; // rad/s
        motorSpeeds[activeMotor].set(MathUtil.applyDeadband(controller.getRightY(), 0.1) * 0.02 * changePerSecond + motorSpeeds[activeMotor].get());

        Motor[] motors = {
            new Motor("Top Flywheel", io.flywheelController, io.topFlywheelMotor),
            new Motor("Bottom Flywheel", bottomFlywheelController, io.bottomFlywheelMotor),
            new Motor("Azimuth", io.azimuthController, io.azimuthMotor),
            new Motor("Hood", io.hoodController, io.hoodMotor)
        };

        Logger.recordOutput("TurretTuning/ActiveMotor", motors[activeMotor].name);

        double totalCurrent = 0.0;
        double totalAverageCurrent = 0.0;

        for(int i = 0; i < motors.length; i++) {
            var motor = motors[i];
            double speed = Units.rotationsPerMinuteToRadiansPerSecond(motorSpeeds[i].get());
            motor.controller.setSetpoint(speed, ControlType.kVelocity);
            
            double current = motor.motor.getOutputCurrent();
            averageSums[i] += current;

            Logger.recordOutput("TurretTuning/AverageCurrent/" + motor.name, averageSums[i] / averageSamples);
            Logger.recordOutput("TurretTuning/Current/" + motor.name, current);
            Logger.recordOutput("TurretTuning/Velocity/" + motor.name, motor.motor.getEncoder().getVelocity());

            totalCurrent += current;
            totalAverageCurrent += averageSums[i] / averageSamples;
        }

        Logger.recordOutput("TurretTuning/TotalCurrent", totalCurrent);
        Logger.recordOutput("TurretTuning/TotalAverageCurrent", totalAverageCurrent);

        averageSamples++;
    }

    public void stop(TurretIOReal io) {
        if(!running) return;
        running = false;
        
        io.configureAndReset();
    }
}
