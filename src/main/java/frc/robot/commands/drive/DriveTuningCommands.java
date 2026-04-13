package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Volts;

import java.io.FileReader;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import java.util.Set;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.google.gson.GsonBuilder;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;
import com.google.gson.Strictness;
import com.google.gson.annotations.Expose;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Container;
import frc.robot.util.Elastic;
import frc.robot.util.GsonClassAdapter;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.tunables.LoggedTunableNumber;

/**
 * A collection of commands for tuning the drive subsystem. All drive tuning commands print their results and save them
 * to a JSON file on the robot.
 */
public class DriveTuningCommands {
    private static final double FF_START_DELAY = 1.0; // Secs
    private static final double FF_RAMP_RATE = 0.2; // Volts/Sec

    private static final LoggedTunableNumber SLIP_START_DELAY = new LoggedTunableNumber("Drive/Tuning/SlipStartDelay", 1.0); // Secs
    private static final LoggedTunableNumber SLIP_START_SETPOINT = new LoggedTunableNumber("Drive/Tuning/SlipStartSetpoint", 20.0); // Amps
    private static final LoggedTunableNumber STATIC_SLIP_RAMP_RATE = new LoggedTunableNumber("Drive/Tuning/StaticSlipRampRate", 1.5); // Amps/Sec
    private static final LoggedTunableNumber DYNAMIC_SLIP_RAMP_RATE = new LoggedTunableNumber("Drive/Tuning/DynamicSlipRampRate", 80.0); // Amps/Sec
    private static final LoggedTunableNumber DYNAMIC_SLIP_RATIO = new LoggedTunableNumber("Drive/Tuning/DynamicSlipRatio", 2.0); // scalar
    private static final LoggedTunableNumber SLIP_TRAVEL_AMOUNT = new LoggedTunableNumber("Drive/Tuning/SlipTravelAmount", Units.degreesToRadians(10)); // Rad

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.5; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public static final double MOI_START_DELAY = 1.0; // Secs
    public static final double MOI_STATIC_CURRENT_ROTATION_SPEED = Units.degreesToRadians(15); // Rad/Sec
    public static final double MOI_STATIC_CURRENT_COLLECTION_TIME = 4.0; // Secs
    public static final double MOI_CURRENT = 10.0; // Amps
    public static final double MOI_MAX_YAW_VEL = Units.degreesToRadians(260); // Rad/Sec

    /** The path to the JSON file where we save our tuning results. */
    public static final String TUNING_RESULTS_FILE = Constants.currentMode == Constants.Mode.REAL
        ? "/U/tuning_results.json" // On a real robot, this is a USB stick
        : "./logs/tuning_results.json"; // In simulation, this is a local file
    /** The path to the JSON file with backup tuning results in case the normal one is missing. */
    public static final String BACKUP_TUNING_RESULTS_FILE = Constants.currentMode == Constants.Mode.REAL
        ? "/C/tuning_results_backup.json" // On a real robot, this is in the home directory
        : "./logs/tuning_results_backup.json"; // In simulation, this is a local file

    // TODO - REALLY TODO - log this and make it work in replay
    /** A set of tuning results that we can load from and save to a JSON file. */
    public static class TuningResults {
        public static class WheelRadiusTuningResults implements LoggableInputs {
            public double radiusMeters;
            public double radiusInches;

            public WheelRadiusTuningResults(double radiusMeters, double radiusInches) {
                this.radiusMeters = radiusMeters;
                this.radiusInches = radiusInches;
            }

            @Override
            public void toLog(LogTable table) {
                table.put("RadiusMeters", radiusMeters);
                table.put("RadiusInches", radiusInches);
            }

            @Override
            public void fromLog(LogTable table) {
                radiusMeters = table.get("RadiusMeters", radiusMeters);
                radiusInches = table.get("RadiusInches", radiusInches);
            }
        }
        public static class FeedforwardTuningResults implements LoggableInputs {
            public double kS;
            public double kV;

            public FeedforwardTuningResults(double kS, double kV) {
                this.kS = kS;
                this.kV = kV;
            }

            @Override
            public void toLog(LogTable table) {
                table.put("kS", kS);
                table.put("kV", kV);
            }

            @Override
            public void fromLog(LogTable table) {
                kS = table.get("kS", kS);
                kV = table.get("kV", kV);
            }
        }
        public static class SlipTuningResults implements LoggableInputs {
            public double slipCurrentAmps;
            public double slipSetpointVolts;
            public double wheelCOF;
            public double[] moduleSlipCurrentsAmps;
            public double[] moduleSlipSetpoints;

            public SlipTuningResults(
                double slipCurrentAmps,
                double slipSetpointVolts,
                double wheelCOF,
                double[] moduleSlipCurrentsAmps,
                double[] moduleSlipSetpoints
            ) {
                this.slipCurrentAmps = slipCurrentAmps;
                this.slipSetpointVolts = slipSetpointVolts;
                this.wheelCOF = wheelCOF;
                this.moduleSlipCurrentsAmps = moduleSlipCurrentsAmps;
                this.moduleSlipSetpoints = moduleSlipSetpoints;
            }
            
            @Override
            public void toLog(LogTable table) {
                table.put("SlipCurrentAmps", slipCurrentAmps);
                table.put("SlipSetpointVolts", slipSetpointVolts);
                table.put("WheelCOF", wheelCOF);
                table.put("ModuleSlipCurrentsAmps", moduleSlipCurrentsAmps);
                table.put("ModuleSlipSetpoints", moduleSlipSetpoints);
            }

            @Override
            public void fromLog(LogTable table) {
                slipCurrentAmps = table.get("SlipCurrentAmps", slipCurrentAmps);
                slipSetpointVolts = table.get("SlipSetpointVolts", slipSetpointVolts);
                wheelCOF = table.get("WheelCOF", wheelCOF);
                moduleSlipCurrentsAmps = table.get("ModuleSlipCurrentsAmps", moduleSlipCurrentsAmps);
                moduleSlipSetpoints = table.get("ModuleSlipSetpoints", moduleSlipSetpoints);
            }
        }
        public static class ModuleZeroingResults implements LoggableInputs {
            public double[] moduleOffsetsRadians;

            public ModuleZeroingResults(double[] moduleOffsetsRadians) {
                this.moduleOffsetsRadians = moduleOffsetsRadians;
            }
            
            @Override
            public void toLog(LogTable table) {
                table.put("ModuleOffsetsRadians", moduleOffsetsRadians);
            }

            @Override
            public void fromLog(LogTable table) {
                moduleOffsetsRadians = table.get("ModuleOffsetsRadians", moduleOffsetsRadians);
            }
        }
        public static class MOIResults implements LoggableInputs {
            public double moiKgM2;

            public MOIResults(double moiKgM2) {
                this.moiKgM2 = moiKgM2;
            }

            @Override
            public void toLog(LogTable table) {
                table.put("MoiKgM2", moiKgM2);
            }

            @Override
            public void fromLog(LogTable table) {
                moiKgM2 = table.get("MoiKgM2", moiKgM2);
            }
        }
        
        public WheelRadiusTuningResults wheelRadiusResults = new WheelRadiusTuningResults(0.0507746, 1.999);
        public FeedforwardTuningResults feedforwardResults = new FeedforwardTuningResults(3.68789, 1.42702);
        public SlipTuningResults slipResults = new SlipTuningResults(
            43.055,
            43.28841825,
            0.9670595939443565,
            new double[] { 69.3, 49.9, 20.240000000000002, 32.78 },
            new double[] { 69.62687700000001, 50.07664199999999, 20.438380000000006, 33.011774 }
        );
        public ModuleZeroingResults moduleZeroingResults = new ModuleZeroingResults(new double[4]);
        public MOIResults moiResults = new MOIResults(0.0);

        // Skip serialization
        @Expose(serialize = false)
        private static GsonBuilder builder = new GsonBuilder()
            .setPrettyPrinting()
            .setStrictness(Strictness.LENIENT) // Read past comments
            .serializeNulls()
            .serializeSpecialFloatingPointValues()
            // Allow serializing records
            .registerTypeAdapter(Class.class, new GsonClassAdapter());

        private void processInputs() {
            Logger.processInputs("Drive/DriveTuningCommands/WheelRadiusResults", wheelRadiusResults);
            Logger.processInputs("Drive/DriveTuningCommands/FeedforwardResults", feedforwardResults);
            Logger.processInputs("Drive/DriveTuningCommands/SlipResults", slipResults);
            Logger.processInputs("Drive/DriveTuningCommands/ModuleZeroingResults", moduleZeroingResults);
            Logger.processInputs("Drive/DriveTuningCommands/MOIResults", moiResults);
        }

        public static TuningResults load() {
            TuningResults results;
            if(!Logger.hasReplaySource()) {
                var file = Filesystem.getOperatingDirectory().toPath().resolve(TUNING_RESULTS_FILE).toFile();
                
                // Make sure the parent directory exists
                file.getParentFile().mkdirs();
                if(!file.exists()) results = loadBackup(); // If the file doesn't exist, return an empty result
                else {
                    var gson = builder.create();
                    try(var fileReader = new FileReader(file)) {
                        results = gson.fromJson(fileReader, TuningResults.class);
                    } catch(JsonSyntaxException | JsonIOException | IOException e) {
                        e.printStackTrace();
                        results = loadBackup(); // If we can't read the file, return an empty result
                    }
                }
            } else {
                results = new TuningResults();
            }
            
            results.processInputs();
            return results;
        }

        private static TuningResults loadBackup() {
            var file = Filesystem.getOperatingDirectory().toPath().resolve(BACKUP_TUNING_RESULTS_FILE).toFile();
            if(!file.exists()) return missingError(); // If the backup file doesn't exist, return an empty result

            var gson = builder.create();
            try(var fileReader = new FileReader(file)) {
                return gson.fromJson(fileReader, TuningResults.class);
            } catch(JsonSyntaxException | JsonIOException | IOException e) {
                e.printStackTrace();
                return missingError(); // If we can't read the backup file, return an empty result
            }
        }

        private static TuningResults missingError() {
            DriverStation.reportError("ERROR: TUNING RESULTS ARE MISSING! EVERYTHING will be broken", false);
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Tuning results", "Tuning results missing! Everything will be broken..."));
            return new TuningResults();
        }

        public String generateReadableResultsComment() {
            if(slipResults.moduleSlipSetpoints == null) slipResults = new SlipTuningResults(
                slipResults.slipCurrentAmps,
                slipResults.slipSetpointVolts,
                slipResults.wheelCOF,
                new double[4],
                new double[4]
            );

            StringBuilder builder = new StringBuilder();
            builder.append("/*");
            
            builder.append("\nTuning Results:\n");
            builder.append(String.format("  Wheel Radius: %.5f meters (%.5f inches)\n",
                wheelRadiusResults.radiusMeters, wheelRadiusResults.radiusInches));
            builder.append(String.format("  Feedforward: kS = %.5f, kV = %.5f\n",
                feedforwardResults.kS, feedforwardResults.kV));
            builder.append(String.format("  Slip Current: %.5f A (Setpoint: %.5f A)\n",
                slipResults.slipCurrentAmps, slipResults.slipSetpointVolts));
            builder.append(String.format("  Estimated Wheel COF: %.5f\n", slipResults.wheelCOF));

            String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };
            builder.append("  Module Slip Currents (A):\n");
            for (int i = 0; i < slipResults.moduleSlipCurrentsAmps.length; i++) {
                builder.append(String.format("    %s: %.5f (Setpoint: %.5f)\n",
                    moduleNames[i],
                    slipResults.moduleSlipCurrentsAmps[i],
                    slipResults.moduleSlipSetpoints[i]));
            }

            builder.append("  Module Zero Offsets (radians):\n");
            for (int i = 0; i < moduleZeroingResults.moduleOffsetsRadians.length; i++) {
                builder.append(String.format("    %s: %.5f\n",
                    moduleNames[i],
                    moduleZeroingResults.moduleOffsetsRadians[i]));
            }
            
            builder.append("*/");
            return builder.toString();
        }

        private transient ArrayList<Runnable> onChangeCallbacks = new ArrayList<>();

        public void save() {
            processInputs();
            if(Logger.hasReplaySource()) return;

            var gson = builder.create();
            try(var fileWriter = new java.io.FileWriter(TUNING_RESULTS_FILE)) {
                gson.toJson(this, fileWriter);
                fileWriter.write("\n");
                fileWriter.write(generateReadableResultsComment());
                System.out.println("Saved tuning results to " + TUNING_RESULTS_FILE);
            } catch(IOException e) {
                e.printStackTrace();
            }

            // Save backup results (no comment)
            try(var fileWriter = new java.io.FileWriter(BACKUP_TUNING_RESULTS_FILE)) {
                gson.toJson(this, fileWriter);
                System.out.println("Saved backup tuning results to " + BACKUP_TUNING_RESULTS_FILE);
            } catch(IOException e) {
                e.printStackTrace();
            }

            for(var callback : onChangeCallbacks) {
                callback.run();
            }
        }

        /** Registers a callback to be run whenever new tuning results are saved. */
        public void nowAndOnChange(Runnable callback) {
            callback.run();
            onChangeCallbacks.add(callback);
        }

        /** Registers a callback to be run whenever new tuning results are saved, but does not run it immediately. */
        public void onChange(Runnable callback) {
            onChangeCallbacks.add(callback);
        }
    }

    private static SysIdRoutine sysIdRoutine = null;
    private static SysIdRoutine angularSysIdRoutine = null;

    private DriveTuningCommands() {
    }

    /** Adds the drive tuning commands to the auto chooser. */
    public static void addTuningCommandsToChooser(Drive drive, LoggedDashboardChooser<Command> chooser) {
        // We might want to run these at a competition
        chooser.addOption("Drive: Rezero Modules (gears climb side)", collectModuleOffsets(drive));

        chooser.addOption("Drive: Wheel Radius Characterization", wheelRadiusCharacterization(drive));
        chooser.addOption("Drive: Static Slip Current Measurement (toward intake)", staticSlipCurrentMeasurement(drive, false));
        chooser.addOption("Drive: Static Slip Current Measurement (away from intake)", staticSlipCurrentMeasurement(drive, true));

        chooser.addOption("Drive: Dynamic Slip Current Measurement (toward intake)", dynamicSlipCurrentMeasurement(drive, true));
        chooser.addOption("Drive: Dynamic Slip Current Measurement (away from intake)", dynamicSlipCurrentMeasurement(drive, false));

        // These only apply to when we're doing "real" tuning
        if(Robot.tuningMode()) {
            chooser.addOption("Drive: Simple FF Characterization", feedforwardCharacterization(drive));
            chooser.addOption("Drive: MOI Characterization", momentOfInertiaCharacterization(drive));

            chooser.addOption("Drive: SysId (Quasistatic Forward)",
                sysIdQuasistatic(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("Drive: SysId (Quasistatic Reverse)",
                sysIdQuasistatic(drive, SysIdRoutine.Direction.kReverse));
            chooser.addOption("Drive: SysId (Dynamic Forward)",
                sysIdDynamic(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("Drive: SysId (Dynamic Reverse)",
                sysIdDynamic(drive, SysIdRoutine.Direction.kReverse));

            chooser.addOption("Drive: Angular SysId (Quasistatic Forward)",
                sysIdQuasistaticAngular(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("Drive: Angular SysId (Quasistatic Reverse)",
                sysIdQuasistaticAngular(drive, SysIdRoutine.Direction.kReverse));
            chooser.addOption("Drive: Angular SysId (Dynamic Forward)",
                sysIdDynamicAngular(drive, SysIdRoutine.Direction.kForward));
            chooser.addOption("Drive: Angular SysId (Dynamic Reverse)",
                sysIdDynamicAngular(drive, SysIdRoutine.Direction.kReverse));
        }
    }

    private static Command require(SubsystemBase subsystem) {
        return subsystem.run(() -> {});
    }

    /** Recollect zero offsets for modules. */
    public static Command collectModuleOffsets(Drive drive) {
        double[] offsetAverages = new double[4];
        Container<Integer> averageSamples = new Container<>(0);
        return Commands.runEnd(() -> {
            for(int i = 0; i < 4; i++) {
                offsetAverages[i] += drive.getZeroOffset(i).getRadians();
            }
            averageSamples.value += 1;
        }, () -> {
            try {
                System.out.println("********** Module Zeroing Results **********");
                for(int i = 0; i < 4; i++) {
                    offsetAverages[i] /= averageSamples.value;
                    Drive.tuningResults.moduleZeroingResults.moduleOffsetsRadians[i] = offsetAverages[i];
                    System.out.println(String.format("Module %d zero offset: %.5f radians", i, offsetAverages[i]));
                }
                System.out.flush();

                Drive.tuningResults.save();
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Module zeroing", "Successfully saved module offsets to moduleOffsets.txt!"));
            } catch(Exception e) {
                e.printStackTrace();
            }
        }, drive);
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        final boolean TUNE_FOR_VOLTAGE = false;
        Timer timer = new Timer();

        if(TUNE_FOR_VOLTAGE) {
            List<Double> velocitySamples = new LinkedList<>();
            List<Double> voltageSamples = new LinkedList<>();

            return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> {
                    drive.runCharacterizationVoltage(0.0);
                }).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(() -> {
                    double setpoint = timer.get() * FF_RAMP_RATE;
                    drive.runCharacterizationVoltage(setpoint);
                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                    voltageSamples.add(setpoint);
                }).finallyDo(() -> { // When cancelled, calculate and print results
                    drive.runCharacterizationVoltage(0.0);

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
                    System.out.println("********** Drive FF Characterization Results For Voltage **********");
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                    System.out.flush();

                    Drive.tuningResults.feedforwardResults = new TuningResults.FeedforwardTuningResults(kS, kV);
                    Drive.tuningResults.save();
                })
            ).alongWith(require(drive));
        } else {
            double[] initialPositions = new double[4];
            List<Double> currentSamples = new ArrayList<>();
            List<Double> results = new ArrayList<>();
            return Commands.sequence(
                // Allow modules to orient
                Commands.run(() -> {
                    drive.runCharacterizationVoltage(0.0);
                }).withTimeout(FF_START_DELAY),
                
                Commands.repeatingSequence(
                    // Start timer
                    Commands.runOnce(() -> {
                        timer.restart();
                        for(int i = 0; i < 4; i++) {
                            initialPositions[i] = drive.getModuleCharacterizationPosition(i);
                        }
                    }),

                    // Accelerate and gather data
                    Commands.run(() -> {
                        double setpoint = timer.get() * FF_RAMP_RATE;
                        drive.runCharacterizationVoltage(setpoint);
                        currentSamples.add(drive.getCharacterizationCurrent());
                    }).until(() -> {
                        double maxChange = 0.0;
                        for(int i = 0; i < 4; i++) {
                            maxChange = Math.max(maxChange, Math.abs(
                                initialPositions[i] - drive.getModuleCharacterizationPosition(i)
                            ));
                        }
                        return maxChange > Units.degreesToRadians(1);
                    }),

                    Commands.runOnce(() -> {
                        // Add result
                        drive.runCharacterizationVoltage(0.0);

                        double setpoint = currentSamples.get(currentSamples.size() - 8);
                        results.add(setpoint);
                    }),
                    
                    // TODO: we could move the wheel a bit between samples to find a better average but like. whatever

                    Commands.waitSeconds(0.5)
                ).withTimeout(30.0)
            ).finallyDo(() -> { // When cancelled, calculate and print results
                drive.runCharacterizationVoltage(0.0);

                double averageResult = results.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                double minResult = results.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);
                double maxResult = results.stream().mapToDouble(Double::doubleValue).max().orElse(0.0);
                
                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results For Voltage **********");
                System.out.println("\tkS: " + formatter.format(averageResult) + " [" + formatter.format(minResult) + " to " + formatter.format(maxResult) + "]");
                System.out.println("\tkV: (always) 0.00");
                System.out.flush();

                Drive.tuningResults.feedforwardResults = new TuningResults.FeedforwardTuningResults(averageResult, 0.);
                Drive.tuningResults.save();
            }).alongWith(require(drive));
        }
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
                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed), false);
                })
            ),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = new double[4];
                    for(int i = 0; i < 4; i++) {
                        state.positions[i] = drive.getModuleCharacterizationPosition(i);
                    }
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
                    var positions = new double[4];
                    for(int i = 0; i < 4; i++) {
                        positions[i] = drive.getModuleCharacterizationPosition(i);
                    }

                    double wheelDelta = 0.0;
                    for(int i = 0; i < 4; i++) wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000");
                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                        + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                    System.out.flush();

                    Drive.tuningResults.wheelRadiusResults = new TuningResults.WheelRadiusTuningResults(wheelRadius, Units.metersToInches(wheelRadius));
                    Drive.tuningResults.save();
                })
            )
        ).alongWith(require(drive));
    }

    private static class SlipCurrentModuleResult {
        public double slipCurrent;
        public double slipSetpoint;
    }

    /**
     * Measures the current at which the robot slips by progressively increasing the wheel voltage and measuring when
     * their velocity jumps. Also estimates the wheel's coefficient of friction. The robot _must_ be placed against a
     * wall for this to work.  
     * Depends on wheel radius tuning.
     */
    public static Command staticSlipCurrentMeasurement(Drive drive, boolean reverseDirection) {
        SlipCurrentModuleResult[] moduleResults = new SlipCurrentModuleResult[4];

        int currentLimitForSlipMeasurement = 100; // Amps

        return Commands.sequence( //
            Commands.runOnce(() -> {
                // Temporarily increase the drive current limit
                drive.setSlipMeasurementCurrentLimit(Amps.of(currentLimitForSlipMeasurement));
                for(int i = 0; i < 4; i++) {
                    moduleResults[i] = new SlipCurrentModuleResult();
                }
            }),

            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterizationCurrent(0.0);
            }).withTimeout(SLIP_START_DELAY.get()),

            Commands.defer(() -> {
                Command[] commands = new Command[4];
                for(int i = 0; i < 4; i++) {
                    commands[i] = slipCurrentWheel(drive, i, moduleResults[i], reverseDirection);
                }
                return Commands.parallel(commands);
            }, Set.of()),

            // Restore the current limit and print results
            Commands.runOnce(() -> {
                drive.setSlipMeasurementCurrentLimit(null);

                double averageSlipCurrent = 0.0;
                double averageSlipVoltage = 0.0;
                for(int i = 0; i < 4; i++) {
                    averageSlipCurrent += moduleResults[i].slipCurrent / 4.;
                    averageSlipVoltage += moduleResults[i].slipSetpoint / 4.;
                }

                NumberFormat formatter = new DecimalFormat("#0.000");

                // Who knows if this means anything physical, but it works (?)
                double[] slipCurrents = new double[] {
                    moduleResults[0].slipCurrent,
                    moduleResults[1].slipCurrent,
                    moduleResults[2].slipCurrent,
                    moduleResults[3].slipCurrent
                };
                slipCurrents = java.util.Arrays.stream(slipCurrents).sorted().toArray();
                double secondHighestSlipCurrnet = slipCurrents[slipCurrents.length - 2];

                System.out.println("********** Drive Slip Current Measurement Results **********");
                System.out.println("\tSlip Current: " + formatter.format(secondHighestSlipCurrnet) + " amps");
                System.out.println("\tAverage slip Current: " + formatter.format(averageSlipCurrent) + " amps");
                System.out.println("\tAverage slip Voltage: " + formatter.format(averageSlipVoltage) + " volts");
                String[] moduleNames = new String[] {
                    "Front left", "Front right", "Back left", "Back right"
                };

                System.out.println("\tIndividual module slip currents:");
                for(int i = 0; i < 4; i++) {
                    System.out.println(
                        "\t \t" + moduleNames[i] + ": " + formatter.format(moduleResults[i].slipCurrent) + " amps");
                }

                // Estimate the wheel's coefficient of friction
                double motorTorque = averageSlipCurrent * DriveConstants.driveMotorModel.KtNMPerAmp;
                double totalTorqueNm = 4 * DriveConstants.driveGearRatio * motorTorque;
                double robotMassN = DriveConstants.robotMass.in(Kilogram) * 9.81;
                double wheelCOF = totalTorqueNm / (robotMassN * Drive.tuningResults.wheelRadiusResults.radiusMeters);
                NumberFormat cofFormatter = new DecimalFormat("#0.0000");
                System.out.println("\tEstimated wheel COF: " + cofFormatter.format(wheelCOF));
                System.out.flush();

                // Save results
                Drive.tuningResults.slipResults = new TuningResults.SlipTuningResults(
                    secondHighestSlipCurrnet,
                    averageSlipVoltage,
                    wheelCOF,
                    new double[] {
                        moduleResults[0].slipCurrent,
                        moduleResults[1].slipCurrent,
                        moduleResults[2].slipCurrent,
                        moduleResults[3].slipCurrent
                    },
                    new double[] {
                        moduleResults[0].slipSetpoint,
                        moduleResults[1].slipSetpoint,
                        moduleResults[2].slipSetpoint,
                        moduleResults[3].slipSetpoint
                    }
                );
                Drive.tuningResults.save();
            })
        ).alongWith(require(drive));
    }

    /**
     * Measures the current at which the robot slips dynamically by accelerating with a constant jerk.  
     * This is fundamentally nearly identical to static slip current measurement but the robot should
     * not be placed near a wall - it should be placed in an open area. The ramp rate should be far higher
     * because otherwise the robot would go like a thousand miles before slipping.
     */
    public static Command dynamicSlipCurrentMeasurement(Drive drive, boolean reverseDirection) {
        List<double[]> currentSamples = new LinkedList<>();
        Timer timer = new Timer();
        Container<Boolean> stopEarly = new Container<Boolean>(false);
        Container<Integer> slippedModule = new Container<>(0);
        double[] moduleAccelerations = new double[4];

        int currentLimitForSlipMeasurement = 130; // Amps

        return Commands.sequence(
            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterizationCurrent(0.0);
            }).withTimeout(SLIP_START_DELAY.get()),

            Commands.runOnce(() -> {
                // Temporarily increase the drive current limit
                drive.setSlipMeasurementCurrentLimit(Amps.of(currentLimitForSlipMeasurement));
                stopEarly.value = false;
                slippedModule.value = 0;
                currentSamples.clear();
                timer.restart();
                for(int i = 0; i < 4; i++) moduleAccelerations[i] = 0;
            }),

            // Accelerate and gather data
            Commands.run(() -> {
                double setpoint = timer.get() * DYNAMIC_SLIP_RAMP_RATE.get() + SLIP_START_SETPOINT.get();
                // if(setpoint > 130) {
                //     System.out.println("No wheels slipped! Capping value.");
                //     stopEarly.value = true;
                // }

                if(reverseDirection) setpoint = -setpoint;
                drive.runCharacterizationCurrent(setpoint);

                double[] currents = new double[4];
                for(int i = 0; i < 4; i++) {
                    currents[i] = drive.getCharacterizationCurrent(i);
                }
                currentSamples.add(currents);
            }).until(() -> {
                if(stopEarly.value) return true;
                if(timer.get() < 0.2) return false;

                // Check if any wheel has slipped by looking for a sudden difference in acceleration
                // compared to the other wheels
                for(int i = 0; i < 4; i++) {
                    double accel = drive.getModuleCharacterizationAcceleration(i);
                    moduleAccelerations[i] = accel;
                }
                
                double[] accelerations = Arrays.stream(moduleAccelerations).sorted().toArray();
                double medianAcceleration = accelerations[1];

                if(medianAcceleration < Units.degreesToRadians(1)) {
                    return false;
                }

                for(int i = 0; i < 4; i++) {
                    double differenceFactor = moduleAccelerations[i] / medianAcceleration;
                    // 3x the median is considered a slip
                    if(differenceFactor > DYNAMIC_SLIP_RATIO.get()) {
                        slippedModule.value = i;
                        System.out.println("Wheel " + i + " slipped at " + moduleAccelerations[i] + " rad/s^2, which is " + differenceFactor + "x the median acceleration of "
                            + medianAcceleration + " rad/s^2");
                        return true;
                    }
                }

                return false;
            }),

            // Reset currents
            Commands.parallel(
                Commands.runOnce(() -> {
                    // skid a bit for fun I guess
                    drive.stopWithX();
                }),

                Commands.run(() -> {
                    double[] currents = new double[4];
                    for(int i = 0; i < 4; i++) {
                        currents[i] = drive.getCharacterizationCurrent(i);
                    }
                    currentSamples.add(currents);
                }).withTimeout(0.05)
            ),

            Commands.waitSeconds(0.2),

            // Take a few samples behind when we stopped and print the result,
            // restore the current limit, and print results
            Commands.runOnce(() -> {
                drive.setSlipMeasurementCurrentLimit(null);
                
                double slipSetpoint = timer.get() * DYNAMIC_SLIP_RAMP_RATE.get() + SLIP_START_SETPOINT.get();

                // Find the maximum current draw of the slipped module
                double slipCurrent = 0.0;
                for(double[] sample : currentSamples) {
                    slipCurrent = Math.max(slipCurrent, sample[slippedModule.value]);
                }

                System.out.println("********** Dynamic Drive Slip Current Measurement Results **********");

                NumberFormat formatter = new DecimalFormat("#0.000");

                if(slipCurrent < 20.) {
                    System.out.println("\tSlip current too low, likely measurement error. Not storing.");
                    System.out.println("\tMeasured slip current: " + formatter.format(slipCurrent) + " amps");
                    System.out.println("\tSetpoint: " + formatter.format(slipSetpoint) + " amps");
                    return;
                }

                // Who knows if this means anything physical, but it works (?)
                String[] moduleNames = new String[] {
                    "Front left", "Front right", "Back left", "Back right"
                };
                System.out.println("\tSlipped module: " + moduleNames[slippedModule.value]);
                System.out.println("\tSlip current: " + formatter.format(slipCurrent) + " amps");
                System.out.println("\tSlip setpoint: " + formatter.format(slipSetpoint) + " amps");

                // Estimate the wheel's coefficient of friction
                double motorTorque = slipCurrent * DriveConstants.driveMotorModel.KtNMPerAmp;
                double totalTorqueNm = 4 * DriveConstants.driveGearRatio * motorTorque;
                double forceOnWheelN = DriveConstants.wheelForceMasses[slippedModule.value].in(Kilogram) * 9.81;
                double wheelCOF = totalTorqueNm / (forceOnWheelN * Drive.tuningResults.wheelRadiusResults.radiusMeters);
                NumberFormat cofFormatter = new DecimalFormat("#0.0000");
                System.out.println("\tEstimated wheel COF: " + cofFormatter.format(wheelCOF));
                System.out.flush();

                // Save results
                Drive.tuningResults.slipResults = new TuningResults.SlipTuningResults(
                    slipCurrent,
                    slipSetpoint,
                    wheelCOF,
                    Drive.tuningResults.slipResults.moduleSlipCurrentsAmps,
                    Drive.tuningResults.slipResults.moduleSlipSetpoints
                );
                Drive.tuningResults.save();
            })
        ).alongWith(require(drive));
    }

    private static Command slipCurrentWheel(Drive drive, int module, SlipCurrentModuleResult moduleResult, boolean reverseDirection) {
        List<Double> currentSamples = new LinkedList<>();
        Timer timer = new Timer();
        Container<Double> startPosition = new Container<>(0.);
        Container<Boolean> stopEarly = new Container<>(false);

        return Commands.sequence(
            Commands.runOnce(() -> {
                currentSamples.clear();
                startPosition.value = drive.getModuleCharacterizationPosition(module);
                stopEarly.value = false;
                timer.restart();
            }),

            // Accelerate and gather data
            Commands.run(() -> {
                double setpoint = timer.get() * STATIC_SLIP_RAMP_RATE.get() + SLIP_START_SETPOINT.get();
                if(setpoint > 90) {
                    System.out.println("Wheel " + module + " didn't slip! Capping value.");
                    stopEarly.value = true;
                }

                if(reverseDirection) setpoint = -setpoint;
                drive.runCharacterizationCurrent(module, setpoint);

                currentSamples.add(drive.getCharacterizationCurrent(module));
            }).until(() -> {
                if(stopEarly.value) return true;

                double distanceTraveled = Math.abs(drive.getModuleCharacterizationPosition(module) - startPosition.value);
                return distanceTraveled > SLIP_TRAVEL_AMOUNT.get();
            }),

            // Take a few samples behind when we stopped and print the result
            Commands.runOnce(() -> {
                drive.runCharacterizationVoltage(module, 0.0);

                moduleResult.slipCurrent = currentSamples.get(currentSamples.size() - 4);
                moduleResult.slipSetpoint = timer.get() * STATIC_SLIP_RAMP_RATE.get() + SLIP_START_SETPOINT.get();

                System.out.println("Module " + module + " slip current measured.");
            }));
    }    

    /** Characterizes the robot MOI. Depends on wheel radius tuning. */
    public static Command momentOfInertiaCharacterization(Drive drive) {
        List<Double> gyroAccelerations = new LinkedList<>();
        List<Double> staticCurrentDraws = new ArrayList<>();
        return Commands.sequence(
            Commands.run(() -> {
                drive.runMOICharacterization(0.0);
            }).withTimeout(MOI_START_DELAY),

            // Briefly turn slowly in place to collect the static current draw
            Commands.run(() -> {
                drive.runVelocity(new ChassisSpeeds(0.0, 0.0, MOI_STATIC_CURRENT_ROTATION_SPEED), false);
                double averageCurrent = 0.0;
                for(int i = 0; i < 4; i++) {
                    averageCurrent += drive.getCharacterizationCurrent(i) / 4.0;
                }
                staticCurrentDraws.add(averageCurrent);
            }).withTimeout(MOI_STATIC_CURRENT_COLLECTION_TIME),

            Commands.run(() -> {
                drive.runMOICharacterization(MOI_CURRENT);
                gyroAccelerations.add(Math.abs(drive.getGyroAcceleration()));
            }).until(() -> Math.abs(drive.getGyroVelocity()) > MOI_MAX_YAW_VEL)
        ).finallyDo(() -> {
            drive.runMOICharacterization(0.0);

            double averageGyroAcceleration = 0.0;
            for(double sample : gyroAccelerations) {
                averageGyroAcceleration += sample / gyroAccelerations.size();
            }

            if(averageGyroAcceleration < 1e-4) {
                System.out.println("Average gyro acceleration too low! Cannot calculate MOI.");
                return;
            }

            // Get the median static current draw
            staticCurrentDraws.sort(Double::compare);
            if(staticCurrentDraws.size() == 0) {
                System.out.println("No static current samples collected! Cannot calculate MOI.");
                return;
            }

            double medianStaticCurrent = staticCurrentDraws.get(staticCurrentDraws.size() / 2);
            double effectiveCurrent = MOI_CURRENT - medianStaticCurrent;

            // TODO: check that this is right? it produces wierd numbers
            double wheelTorque = DriveConstants.driveGearRatio * effectiveCurrent * DriveConstants.driveMotorModel.KtNMPerAmp;
            double wheelForceAtGround = wheelTorque / Drive.tuningResults.wheelRadiusResults.radiusMeters;
            double torqueOnRobot = wheelForceAtGround * DriveConstants.driveBaseRadius * 4;
            double moi = torqueOnRobot / averageGyroAcceleration;

            NumberFormat formatter = new DecimalFormat("#0.000");
            System.out.println("********** Drive MOI Characterization Results **********");
            System.out.println("\tAverage gyro acceleration: " + formatter.format(averageGyroAcceleration) + " radians/s^2");
            System.out.println("\tEstimated MOI: " + formatter.format(moi) + " kg*m^2");
            System.out.flush();

            Drive.tuningResults.moiResults = new TuningResults.MOIResults(moi);
        }).alongWith(require(drive));
    }

    /** Configures the SysId routine if it hasn't been configured yet. */
    private static void initSysId(Drive drive) {
        if(sysIdRoutine == null) {
            sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> drive.runCharacterizationCurrent(voltage.in(Volts)), null, drive));
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public static Command sysIdQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
        initSysId(drive);

        return Commands.run(() -> drive.runCharacterizationCurrent(0.0), drive).withTimeout(1.0)
            .andThen(sysIdRoutine.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public static Command sysIdDynamic(Drive drive, SysIdRoutine.Direction direction) {
        initSysId(drive);

        return Commands.run(() -> drive.runCharacterizationCurrent(0.0), drive).withTimeout(1.0)
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
