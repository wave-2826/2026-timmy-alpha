package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.io.FileReader;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import java.util.Set;
import java.util.LinkedList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.google.gson.GsonBuilder;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Container;

/**
 * A collection of commands for tuning the drive subsystem. All drive tuning commands print their results and save them
 * to a JSON file on the robot.
 */
public class DriveTuningCommands {
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private static final double SLIP_START_DELAY = 1.0; // Secs
    private static final double SLIP_START_VOLTAGE = 0.4; // Volts
    private static final double SLIP_RAMP_RATE = 0.075; // Volts/Sec
    private static final double SLIP_TRAVEL_AMOUNT = Units.degreesToRadians(15); // Rad

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    /** The path to the JSON file where we save our tuning results. */
    public static final String TUNING_RESULTS_FILE = Constants.currentMode == Constants.Mode.REAL
        ? "/U/tuning_results.json" // On a real robot, this is a USB stick
        : "./logs/tuning_results.json"; // In simulation, this is a local file

    /** A set of tuning results that we can load from and save to a JSON file. */
    @SuppressWarnings("unused") // This is used for serialization and deserialization
    private static class TuningResults {
        public double wheelRadiusMeters = 0.0; // Meters
        public double kS = 0.0; // Volts
        public double kV = 0.0; // Volts/(m/s)
        public double slipCurrentAmps = 0.0; // Amps
        public double slipVoltageVolts = 0.0; // Volts
        public double wheelCOF = 0.0; // Coefficient of friction
        public double[] moduleSlipCurrentsAmps = new double[4]; // Amps
        public double[] moduleSlipVoltagesVolts = new double[4]; // Volts

        public static TuningResults load() {
            var file = Filesystem.getOperatingDirectory().toPath().resolve(TUNING_RESULTS_FILE).toFile();
            // Make sure the parent directory exists
            file.getParentFile().mkdirs();

            if(!file.exists()) return new TuningResults(); // If the file doesn't exist, return an empty result

            var builder = new GsonBuilder();
            builder.setPrettyPrinting();
            var gson = builder.create();
            try(var fileReader = new FileReader(file)) {
                return gson.fromJson(fileReader, TuningResults.class);
            } catch(JsonSyntaxException | JsonIOException | IOException e) {
                e.printStackTrace();
                return new TuningResults(); // If we can't read the file, return an empty result
            }
        }

        public void save() {
            var builder = new GsonBuilder();
            builder.setPrettyPrinting();
            var gson = builder.create();
            try(var fileWriter = new java.io.FileWriter(TUNING_RESULTS_FILE)) {
                gson.toJson(this, fileWriter);
                System.out.println("Saved tuning results to " + TUNING_RESULTS_FILE);
            } catch(IOException e) {
                e.printStackTrace();
            }
        }
    }

    /** The tuning results that we can load from and save to a JSON file. */
    private static TuningResults tuningResults = TuningResults.load();

    private static SysIdRoutine sysIdRoutine = null;
    private static SysIdRoutine angularSysIdRoutine = null;

    private DriveTuningCommands() {
    }

    /** Adds the drive tuning commands to the auto chooser. */
    public static void addTuningCommandsToAutoChooser(Drive drive, LoggedDashboardChooser<Command> chooser) {
        // We might want to run these at a competition
        chooser.addOption("TUNING | Drive Wheel Radius Characterization", wheelRadiusCharacterization(drive));
        chooser.addOption("TUNING | Drive Slip Current Measurement", slipCurrentMeasurement(drive));

        // These only apply to when we're doing "real" tuning
        if(Constants.tuningMode) {
            chooser.addOption("TUNING | Drive Simple FF Characterization", feedforwardCharacterization(drive));

            chooser.addOption("TUNING | Drive SysId (Quasistatic Forward)",
                sysIdQuasistatic(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Drive SysId (Quasistatic Reverse)",
                sysIdQuasistatic(drive, SysIdRoutine.Direction.kReverse));
            chooser.addOption("TUNING | Drive SysId (Dynamic Forward)",
                sysIdDynamic(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Drive SysId (Dynamic Reverse)",
                sysIdDynamic(drive, SysIdRoutine.Direction.kReverse));

            chooser.addOption("TUNING | Drive Angular SysId (Quasistatic Forward)",
                sysIdQuasistaticAngular(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Drive Angular SysId (Quasistatic Reverse)",
                sysIdQuasistaticAngular(drive, SysIdRoutine.Direction.kReverse));
            chooser.addOption("TUNING | Drive Angular SysId (Dynamic Forward)",
                sysIdDynamicAngular(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Drive Angular SysId (Dynamic Reverse)",
                sysIdDynamicAngular(drive, SysIdRoutine.Direction.kReverse));
        }
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterization(0.0);
            }, drive).withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }, drive).finallyDo(() -> { // When cancelled, calculate and print results
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for(int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));

                tuningResults.kS = kS;
                tuningResults.kV = kV;
                tuningResults.save();
            }));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
        RobotState robotState = RobotState.getInstance();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> {
                    limiter.reset(0.0);
                }),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                }, drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = robotState.getRotation();
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    var rotation = robotState.getRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                })

                    // When cancelled, calculate and print results
                    .finallyDo(() -> {
                        double[] positions = drive.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for(int i = 0; i < 4; i++) wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                        NumberFormat formatter = new DecimalFormat("#0.000");
                        System.out.println("********** Wheel Radius Characterization Results **********");
                        System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                        System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                        System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                            + formatter.format(Units.metersToInches(wheelRadius)) + " inches");

                        tuningResults.wheelRadiusMeters = wheelRadius;
                        tuningResults.save();
                    })));
    }

    private static class SlipCurrentModuleResult {
        public double slipCurrent;
        public double slipVoltage;
    }

    /**
     * Measures the current at which the robot slips by progressively increasing the wheel voltage and measuring when
     * their velocity jumps. Also estimates the wheel's coefficient of friction. The robot _must_ be placed against a
     * wall for this to work.
     */
    public static Command slipCurrentMeasurement(Drive drive) {
        SlipCurrentModuleResult[] moduleResults = new SlipCurrentModuleResult[4];

        int currentLimitForSlipMeasurement = 100; // Amps

        Command command = Commands.sequence( //
            Commands.runOnce(() -> {
                // Temporarily increase the drive current limit
                drive.setSlipMeasurementCurrentLimit(Amps.of(currentLimitForSlipMeasurement));
                for(int i = 0; i < 4; i++) {
                    moduleResults[i] = new SlipCurrentModuleResult();
                }
            }),

            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterization(0.0);
            }).withTimeout(SLIP_START_DELAY),

            Commands.defer(() -> {
                Command[] commands = new Command[4];
                for(int i = 0; i < 4; i++) {
                    commands[i] = slipCurrentWheel(drive, i, moduleResults[i]);
                }
                return Commands.parallel(commands);
            }, Set.of()),

            // Restore the current limit and print results
            Commands.runOnce(() -> {
                drive.setSlipMeasurementCurrentLimit(DriveConstants.slipCurrent);

                double averageSlipCurrent = 0.0;
                double averageSlipVoltage = 0.0;
                for(int i = 0; i < 4; i++) {
                    averageSlipCurrent += moduleResults[i].slipCurrent / 4.;
                    averageSlipVoltage += moduleResults[i].slipVoltage / 4.;
                }

                NumberFormat formatter = new DecimalFormat("#0.000");

                System.out.println("********** Drive Slip Current Measurement Results **********");
                System.out.println("\tAverage slip Current: " + formatter.format(averageSlipCurrent) + " amps");
                System.out.println("\tAverage slip \"Voltage\": " + formatter.format(averageSlipVoltage) + " volts");
                String[] moduleNames = new String[] {
                    "Front left", "Front right", "Back left", "Back right"
                };

                System.out.println("\tIndividual module slip currents:");
                for(int i = 0; i < 4; i++) {
                    System.out.println(
                        "\t \t" + moduleNames[i] + ": " + formatter.format(moduleResults[i].slipCurrent) + " amps");
                }

                // Estimate the wheel's coefficient of friction
                double motorTorque = averageSlipCurrent * DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
                double totalTorqueNm = 4 * DriveConstants.driveGearRatio * motorTorque;
                double robotMassN = DriveConstants.robotMass.in(Kilogram) * 9.81;
                double wheelCOF = totalTorqueNm / (robotMassN * DriveConstants.wheelRadius.in(Meters));
                NumberFormat cofFormatter = new DecimalFormat("#0.0000");
                System.out.println("\tEstimated wheel COF: " + cofFormatter.format(wheelCOF));

                // Save results
                tuningResults.slipCurrentAmps = averageSlipCurrent;
                tuningResults.slipVoltageVolts = averageSlipVoltage;
                tuningResults.wheelCOF = wheelCOF;
                for(int i = 0; i < 4; i++) {
                    tuningResults.moduleSlipCurrentsAmps[i] = moduleResults[i].slipCurrent;
                    tuningResults.moduleSlipVoltagesVolts[i] = moduleResults[i].slipVoltage;
                }
                tuningResults.save();
            })
        );
        command.addRequirements(drive);
        return command;
    }

    private static Command slipCurrentWheel(Drive drive, int module, SlipCurrentModuleResult moduleResult) {
        List<Double> currentSamples = new LinkedList<>();
        Timer timer = new Timer();
        Container<Double> startPosition = new Container<Double>(0.);

        return Commands.sequence(Commands.runOnce(() -> {
            currentSamples.clear();
            startPosition.value = drive.getSlipMeasurementPosition(module);
            timer.restart();
        }),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = Math.min(12., timer.get() * SLIP_RAMP_RATE + SLIP_START_VOLTAGE);
                drive.runCharacterization(module, voltage);

                currentSamples.add(drive.getSlipMeasurementCurrent(module));
            }).until(() -> {
                if(timer.get() * SLIP_RAMP_RATE + SLIP_START_VOLTAGE > 12.) {
                    System.out.println("Slip current measurement capped at 12 volts. This probably isn't correct.");
                    return true; // Stop if we hit the voltage limit
                }

                double distanceTraveled = Math.abs(drive.getSlipMeasurementPosition(module) - startPosition.value);
                return distanceTraveled > SLIP_TRAVEL_AMOUNT;
            }),

            // Take a few samples behind when we stopped and print the result
            Commands.runOnce(() -> {
                drive.runCharacterization(module, 0.0);

                moduleResult.slipCurrent = currentSamples.get(currentSamples.size() - 4);
                moduleResult.slipVoltage = timer.get() * SLIP_RAMP_RATE + SLIP_START_VOLTAGE;

                System.out.println("Module " + module + " slip current measured.");
            }));
    }

    /** Configures the SysId routine if it hasn't been configured yet. */
    private static void initSysId(Drive drive) {
        if(sysIdRoutine == null) {
            sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> drive.runCharacterization(voltage.in(Volts)), null, drive));
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public static Command sysIdQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
        initSysId(drive);

        return Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(1.0)
            .andThen(sysIdRoutine.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public static Command sysIdDynamic(Drive drive, SysIdRoutine.Direction direction) {
        initSysId(drive);

        return Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(1.0)
            .andThen(sysIdRoutine.dynamic(direction));
    }

    /** Configures the angular SysId routine if it hasn't been configured yet. */
    private static void initAngularSysId(Drive drive) {
        if(angularSysIdRoutine == null) {
            angularSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> drive.runAngularCharacterization(voltage.in(Volts)), null,
                    drive));
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public static Command sysIdQuasistaticAngular(Drive drive, SysIdRoutine.Direction direction) {
        initAngularSysId(drive);

        return Commands.run(() -> drive.runAngularCharacterization(0.0), drive).withTimeout(1.0)
            .andThen(angularSysIdRoutine.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public static Command sysIdDynamicAngular(Drive drive, SysIdRoutine.Direction direction) {
        initAngularSysId(drive);

        return Commands.run(() -> drive.runAngularCharacterization(0.0), drive).withTimeout(1.0)
            .andThen(angularSysIdRoutine.dynamic(direction));
    }
}
