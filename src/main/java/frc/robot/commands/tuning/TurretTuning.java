package frc.robot.commands.tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.FileWriter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * This class does NOT work for replay! It's only for testing and tuning purposes.
 */
public class TurretTuning {
    private int activeMotor = 0;

    private static double ENUMERATION_PERIOD = 2.5;

    /** File handle for the CSV file we're writing to. */
    FileWriter enumerationDataFile;

    public static void init() {
        // Run static initialization lol   
    }

    private TurretIO io;
    private CommandXboxController controller;
    private class Motor {
        String name;
        DoubleSupplier speedSupplier;
        DoubleSupplier currentSupplier;
        SlewRateLimiter speedLimiter;
        double speedTarget = 0.0; // rad/s
        double manualSpeed = 0.0;

        public Motor(String name, DoubleSupplier speedSupplier, DoubleSupplier currentSupplier, double slewRateRPMPS) {
            this.name = name;
            this.speedSupplier = speedSupplier;
            this.currentSupplier = currentSupplier;
            this.speedLimiter = new SlewRateLimiter(Units.rotationsPerMinuteToRadiansPerSecond(slewRateRPMPS));
        }

        /** rad/sec */
        public void setSetpoint(double speedRPM) {
            speedTarget = Units.rotationsPerMinuteToRadiansPerSecond(speedRPM);
        }
        /** rad/sec */
        public double getTrueSetpoint() {
            return speedLimiter.calculate(speedTarget);
        }
    }
    private Motor[] motors;
    private EventLoop triggerLoop = new EventLoop();
    
    /** Inputs shouldn't change but it's weird */
    public TurretTuning(TurretIO io, Supplier<TurretIOInputs> inputs, CommandXboxController controller) {
        this.io = io;
        this.controller = controller;
        this.motors = new Motor[] {
            new Motor("Flywheel", () -> inputs.get().flywheel1.velocityRadPerSec(), () -> inputs.get().flywheel1.currentAmps(), 4000),
            new Motor("Hood", () -> inputs.get().hood.velocityRadPerSec(), () -> inputs.get().hood.currentAmps(), 4000),
            new Motor("Azimuth", () -> inputs.get().azimuth.internalEncoderVelocity(), () -> inputs.get().azimuth.currentAmps(), 600)
        };
    }

    public void start() {
        controller.rightBumper(triggerLoop).onTrue(Commands.runOnce(this::nextMotor));
        controller.leftBumper(triggerLoop).onTrue(Commands.runOnce(this::previousMotor));
        controller.a(triggerLoop).onTrue(Commands.runOnce(this::resetAverage));
        controller.b(triggerLoop).onTrue(Commands.runOnce(this::toggleEnumerateSequence));
    }

    private void nextMotor() {
        activeMotor = (activeMotor + 1) % motors.length;
    }
    private void previousMotor() {
        activeMotor = (activeMotor - 1 + motors.length) % motors.length;
    }

    private void resetAverage() {
        averageSamples = 0;
        for(int i = 0; i < currentAverageSums.length; i++) {
            currentAverageSums[i] = 0.0;
            velocityAverageSums[i] = 0.0;
        }
    }


    /** The speeds enumerated on each motor; the total number of combinations is N^3 where N is the length of this array. */
    private double[] azimuthEnumerationSpeeds = new double[] { 0, 50, 100, 150, 200 };
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
        return speeds;
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

    private int averageSamples = 0;
    private double[] currentAverageSums = new double[4];
    private double[] velocityAverageSums = new double[4];

    /** Get the specified motor speed in RPM */
    private double getMotorSpeed(int motorIndex) {
        if(enumerating) {
            double[] speeds = getSpeeds(enumerationIndex);
            return speeds[motorIndex];
        } else {
            return motors[motorIndex].manualSpeed;
        }
    }

    public void run() {
        triggerLoop.poll();

        double changePerSecond = -200; // rpm/s
        motors[activeMotor].manualSpeed += MathUtil.applyDeadband(controller.getRightY(), 0.1) * 0.02 * changePerSecond;

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
            if(timeIntoCurrentStep < 0.2) {
                resetAverage();
            }
        }

        for(int i = 0; i < motors.length; i++) {
            var motor = motors[i];

            double motorSpeed = getMotorSpeed(i);
            motor.setSetpoint(motorSpeed);
            
            double current = motor.currentSupplier.getAsDouble();
            double velocity = motor.speedSupplier.getAsDouble();

            currentAverageSums[i] += current;
            velocityAverageSums[i] += velocity;

            Logger.recordOutput("TurretTuning/AverageCurrent/" + motor.name, currentAverageSums[i] / averageSamples, Amps);
            Logger.recordOutput("TurretTuning/Current/" + motor.name, current, Amps);
            Logger.recordOutput("TurretTuning/Velocity/" + motor.name, velocity, RadiansPerSecond);
            Logger.recordOutput("TurretTuning/VelocityTarget/" + motor.name, Units.rotationsPerMinuteToRadiansPerSecond(motorSpeed), RadiansPerSecond); // rad/sec

            totalCurrent += current;
            totalAverageCurrent += currentAverageSums[i] / averageSamples;
        }

        io.setVelocityOutputs(
            motors[0].getTrueSetpoint(), // fly
            motors[2].getTrueSetpoint(), // azimuth
            motors[1].getTrueSetpoint()  // hood
        );

        Logger.recordOutput("TurretTuning/TotalCurrent", totalCurrent, Amps);
        Logger.recordOutput("TurretTuning/TotalAverageCurrent", totalAverageCurrent, Amps);

        averageSamples++;
    }

    public void stop() {
        if(enumerating) stopEnumeration();

        triggerLoop.clear();
    }
}
