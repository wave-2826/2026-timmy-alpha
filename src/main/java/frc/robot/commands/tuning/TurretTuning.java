package frc.robot.commands.tuning;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.turret.TurretIOSpark;
import frc.robot.util.Elastic;
import frc.robot.util.SparkPIDConstants;
import frc.robot.util.SparkUtil;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.tunables.TunableSparkPID;

import static frc.robot.util.SparkUtil.tryUntilOk;

import java.io.FileWriter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class does NOT work for replay! It's only for testing and tuning purposes.
 */
public class TurretTuning {
    private int activeMotor = 0;

    private static double ENUMERATION_PERIOD = 2.5;

    private static TunableSparkPID flywheelVelocityPID = new TunableSparkPID("TurretTuning/FlywheelPID")
        .addRealRobotGains(new SparkPIDConstants(0.001, 0.0, 0.0, 0.02, ClosedLoopSlot.kSlot1));
    private static TunableSparkPID azimuthVelocityPID = new TunableSparkPID("TurretTuning/AzimuthPID")
        .addRealRobotGains(new SparkPIDConstants(0.008, 0.0, 0.0, 0.025, ClosedLoopSlot.kSlot1));

    /** File handle for the CSV file we're writing to. */
    FileWriter enumerationDataFile;

    public static void init() {
        // Run static initialization lol   
    }

    TurretIOSpark io;
    CommandXboxController controller;
    private class Motor {
        SparkClosedLoopController controller;
        String name;
        SparkBase motor;
        public Motor(String name, SparkBase motor) {
            this.name = name;
            this.motor = motor;
            this.controller = motor.getClosedLoopController();
        }
        
    }
    private Motor[] motors;
    private EventLoop triggerLoop = new EventLoop();
    
    public TurretTuning(TurretIOSpark io, CommandXboxController controller) {
        this.io = io;
        this.controller = controller;
        this.motors = new Motor[] {
            new Motor("Top Flywheel", io.topFlywheelMotor),
            new Motor("Bottom Flywheel", io.bottomFlywheelMotor),
            new Motor("Azimuth", io.azimuthMotor),
            new Motor("Hood", io.hoodMotor)
        };
    }

    public void start() {
        var testMotorConfig = new SparkFlexConfig();
        testMotorConfig.signals.outputCurrentPeriodMs(20).primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20);
        testMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        testMotorConfig.encoder
            .positionConversionFactor(2.0 * Math.PI) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0) // RPM -> rad/s
            .uvwAverageDepth(8)
            .uvwMeasurementPeriod(32);
        testMotorConfig.disableFollowerMode();
        testMotorConfig.closedLoopRampRate(0.5);
        
        var testAzimuthConfig = new SparkFlexConfig().apply(testMotorConfig);

        flywheelVelocityPID.applyConfigAndRegister(testMotorConfig, io.topFlywheelMotor, io.bottomFlywheelMotor, io.hoodMotor);
        tryUntilOk(io.topFlywheelMotor, 5, () -> io.topFlywheelMotor.configure(testMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        
        var bottomFlywheelConfig = new SparkFlexConfig().apply(testMotorConfig).inverted(true);
        tryUntilOk(io.bottomFlywheelMotor, 5, () -> io.bottomFlywheelMotor.configure(bottomFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        tryUntilOk(io.hoodMotor, 5, () -> io.hoodMotor.configure(testMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        
        azimuthVelocityPID.applyConfigAndRegister(testAzimuthConfig, io.azimuthMotor);
        tryUntilOk(io.azimuthMotor, 5, () -> io.azimuthMotor.configure(testAzimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

        if(SparkUtil.checkFault()) {
            Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "Turret tuning", "Failed to configure"));
        } else {
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Turret tuning", "Successfully reconfigured"));
        }

        controller.rightBumper(triggerLoop).onTrue(Commands.runOnce(this::nextMotor));
        controller.leftBumper(triggerLoop).onTrue(Commands.runOnce(this::previousMotor));
        controller.a(triggerLoop).onTrue(Commands.runOnce(this::resetAverage));
        controller.b(triggerLoop).onTrue(Commands.runOnce(this::toggleEnumerateSequence));
    }

    private void nextMotor() {
        activeMotor = (activeMotor + 1) % 4;
    }
    private void previousMotor() {
        activeMotor = (activeMotor - 1 + 4) % 4;
    }

    private void resetAverage() {
        averageSamples = 0;
        for(int i = 0; i < currentAverageSums.length; i++) {
            currentAverageSums[i] = 0.0;
            velocityAverageSums[i] = 0.0;
        }
    }


    /** The speeds enumerated on each motor; the total number of combinations is N^3 where N is the length of this array. */
    private double[] azimuthEnumerationSpeeds = new double[] { 0, 200, 500, 750, 1000 };
    private double[] flywheelEnumerationSpeeds = new double[] { 0, 250, 1000, 1750, 2500 };

    /**
     * Gets the Nth combination of enumerationSpeeds for each motor,
     * where the motors are in the order flywheel, hood, azimuth (so azimuth changes the least).
     */
    private double[] getSpeeds(int n) {
        double[] speeds = new double[]{ 0, 0, 0 };
        double[][] enumerationSpeeds = new double[][] { flywheelEnumerationSpeeds, flywheelEnumerationSpeeds, azimuthEnumerationSpeeds };

        for(int i = 0; i < 3; i++) {
            int index = (n / (int)Math.pow(flywheelEnumerationSpeeds.length, i)) % flywheelEnumerationSpeeds.length;
            speeds[i] = enumerationSpeeds[i][index];
        }
        return new double[] { speeds[0], speeds[0], speeds[2], speeds[1] };
    }
    
    private double enumerationStartTime = 0;
    private boolean enumerating = false;
    private int enumerationIndex = 0;

    private void toggleEnumerateSequence() {
        enumerating = !enumerating;

        if(enumerating) {
            beginEnumeration();
        } else {
            stopEnumeration();
        }
    }

    private void beginEnumeration() {
        enumerationStartTime = Timer.getFPGATimestamp();
        enumerationIndex = -1;
        
        // Create a CSV file in the deploy directory
        try {
            String filename = "/U/turret_enumeration_data_" + System.currentTimeMillis() + ".csv";
            enumerationDataFile = new FileWriter(filename, true);
            StringBuilder titles = new StringBuilder();
            for(var motor : motors) {
                titles.append(motor.name).append(" Target Velocity,").append(motor.name).append(" Measured Velocity,").append(motor.name).append(" Current,");
            }
            enumerationDataFile.write(titles.toString() + "\n");
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Turret tuning", "Started enumeration logging to " + filename));
        } catch(Exception e) {
            e.printStackTrace();
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Turret tuning", "Failed to start enumeration"));
        }
    }
    private void logEnumerationLine() {
        if(enumerationDataFile == null) return;

        try {
            StringBuilder line = new StringBuilder();
            for(int i = 0; i < motors.length; i++) {
                double targetVelocity = Units.rotationsPerMinuteToRadiansPerSecond(getMotorSpeed(i));

                double current = currentAverageSums[i] / averageSamples;
                double measuredVelocity = velocityAverageSums[i] / averageSamples;
                
                line.append(targetVelocity).append(",").append(measuredVelocity).append(",").append(current).append(",");
            }
            enumerationDataFile.write(line.toString().replaceAll(",$", "") + "\n");
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Turret tuning", "Step " + enumerationIndex + "/" + (int)Math.pow(azimuthEnumerationSpeeds.length, 3) + " logged"));
        } catch(Exception e) {
            e.printStackTrace();
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Turret tuning", "Failed to write data"));
        }
    }
    private void stopEnumeration() {
        if(enumerationDataFile == null) return;

        // Close the data file
        try {
            enumerationDataFile.close();
        } catch(Exception e) {
            e.printStackTrace();
            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Turret tuning", "Failed to close data"));
        }
        enumerationDataFile = null;
    }

    private static LoggedNetworkNumber[] motorSpeeds = {
        new LoggedNetworkNumber("Tuning/TurretTuning/TopFlywheelVelocity", 0.0),
        new LoggedNetworkNumber("Tuning/TurretTuning/BottomFlywheelVelocity", 0.0),
        new LoggedNetworkNumber("Tuning/TurretTuning/AzimuthVelocity", 0.0),
        new LoggedNetworkNumber("Tuning/TurretTuning/HoodVelocity", 0.0)
    };

    private int averageSamples = 0;
    private double[] currentAverageSums = new double[4];
    private double[] velocityAverageSums = new double[4];

    private double getMotorSpeed(int motorIndex) {
        if(enumerating) {
            double[] speeds = getSpeeds(enumerationIndex);
            return speeds[motorIndex];
        } else {
            return motorSpeeds[motorIndex].get();
        }
    }

    public void run() {
        triggerLoop.poll();

        double changePerSecond = -200; // rpm/s
        motorSpeeds[activeMotor].set(MathUtil.applyDeadband(controller.getRightY(), 0.1) * 0.02 * changePerSecond + motorSpeeds[activeMotor].get());

        Logger.recordOutput("TurretTuning/ActiveMotor", motors[activeMotor].name);

        double totalCurrent = 0.0;
        double totalAverageCurrent = 0.0;

        if(enumerating && Timer.getFPGATimestamp() - enumerationStartTime > ENUMERATION_PERIOD * Math.pow(azimuthEnumerationSpeeds.length, 3)) {
            enumerating = false;
            stopEnumeration();
        }
        // Reset average period at the start of every cycle
        if(enumerating) {
            int newEnumerationIndex = (int)((Timer.getFPGATimestamp() - enumerationStartTime) / ENUMERATION_PERIOD);
            if(newEnumerationIndex != enumerationIndex) {
                if(enumerationIndex > 0) {
                    logEnumerationLine();
                }
            }
            enumerationIndex = newEnumerationIndex;

            double timeIntoCurrentStep = (Timer.getFPGATimestamp() - enumerationStartTime) % ENUMERATION_PERIOD;
            // Reset averages halfway through each step once the motors have had time to settle
            if(timeIntoCurrentStep < 0.02) {
                resetAverage();
            }
        }

        for(int i = 0; i < motors.length; i++) {
            var motor = motors[i];

            double motorSpeed = getMotorSpeed(i);

            double speed = Units.rotationsPerMinuteToRadiansPerSecond(motorSpeed);
            motor.controller.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
            
            double current = motor.motor.getOutputCurrent();
            double velocity = motor.motor.getEncoder().getVelocity();

            currentAverageSums[i] += current;
            velocityAverageSums[i] += velocity;

            Logger.recordOutput("TurretTuning/AverageCurrent/" + motor.name, currentAverageSums[i] / averageSamples);
            Logger.recordOutput("TurretTuning/Current/" + motor.name, current);
            Logger.recordOutput("TurretTuning/Velocity/" + motor.name, velocity);
            Logger.recordOutput("TurretTuning/VelocityTarget/" + motor.name, speed);

            totalCurrent += current;
            totalAverageCurrent += currentAverageSums[i] / averageSamples;
        }

        Logger.recordOutput("TurretTuning/TotalCurrent", totalCurrent);
        Logger.recordOutput("TurretTuning/TotalAverageCurrent", totalAverageCurrent);

        averageSamples++;
    }

    public void stop() {
        io.configureAndReset();

        if(enumerating) stopEnumeration();

        triggerLoop.clear();
    }
}
