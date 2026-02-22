package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Controls;
import frc.robot.commands.tuning.TurretTuning;
import frc.robot.subsystems.turret.TurretIO.TurretIOMPCOutputs;
import frc.robot.subsystems.turret.TurretIO.TurretIOPIDOutputs;

/**
 * Our robot has a triple-coaxial turret - all motors are static relative to the robot frame.  
 * The power transmission stack is as follows:
 * - Flywheel: 2x NEO Vortex, on the "innermost" coaxial stage; this will be affected by the azimuth rotation, but runs
 *   at a high velocity anyway so we don't care to compensate. The motors spin opposite, and the top needs to spin counterclockwise
 *   to shoot.
 * - Hood: 1x NEO Vortex, on the "middle" coaxial stage. Must run with the azimuth rotation to maintain a consistent hood angle.
 * - Azimuth: 1x NEO Vortex, on the "outermost" coaxial stage. Isn't affected by the other two stages and runs closed-loop over an
 *   attached absolute encoder.
 */
public class Turret extends SubsystemBase {
    public class TurretTarget {
        public double flywheelSpeedRadPerSec;
        public double azimuthAngleRad;
        public double hoodAngleRad;

        public TurretTarget(double flywheelSpeedRadPerSec, double azimuthAngleRad, double hoodRingAngleDiffRad) {
            this.flywheelSpeedRadPerSec = flywheelSpeedRadPerSec;
            this.azimuthAngleRad = azimuthAngleRad;
            this.hoodAngleRad = hoodRingAngleDiffRad;
        }
    }
    private enum ControlMode {
        NONE,
        PID,
        MPC
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private LoggedDashboardChooser<ControlMode> controlModeChooser = new LoggedDashboardChooser<>("Turret/ControlMode");

    public TurretTarget target = null;

    private TurretController controller;

    public Turret(TurretIO io) {
        this.io = io;
        controller = new TurretController(inputs);

        controlModeChooser.addDefaultOption("PID", ControlMode.PID);
        controlModeChooser.addOption("MPC", ControlMode.MPC);

        TurretTuning.init();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if(DriverStation.isTest()) return;

        if(target == null) {
            io.stop();
        } else {
            switch(controlModeChooser.get()) {
                case NONE:
                    return;
                case PID: {
                    TurretIOPIDOutputs outputs = new TurretIOPIDOutputs(
                        target.flywheelSpeedRadPerSec,
                        target.azimuthAngleRad % (Math.PI * 2),
                        MathUtil.clamp(target.hoodAngleRad, 0, Math.PI / 2)
                    );
                    io.setPIDOutputs(outputs);
                    break;
                }
                case MPC: {
                    double[] outputs = controller.getOutputs(target.azimuthAngleRad, target.hoodAngleRad, target.flywheelSpeedRadPerSec);
                    TurretIOMPCOutputs mpcOutputs = new TurretIOMPCOutputs(
                        outputs[0],
                        outputs[1],
                        outputs[2]
                    );
                    io.setMPCOutputs(mpcOutputs);
                    break;
                }
            }

            Logger.recordOutput("Turret/Target/Azimuth", target.azimuthAngleRad);
            Logger.recordOutput("Turret/Target/Hood", target.hoodAngleRad);
            Logger.recordOutput("Turret/Target/Flywheel", target.flywheelSpeedRadPerSec);
        }
    }

    public Command runManual(
        DoubleSupplier flywheelSpeedSupplier,
        DoubleSupplier azimuthSpeedSupplier,
        DoubleSupplier hoodSpeedSupplier
    ) {
        return Commands.runEnd(() -> {
            if(target == null) {
                target = new TurretTarget(0.0, inputs.azimuth.azimuthAngleRad(), 0);
            }

            target.flywheelSpeedRadPerSec = MathUtil.applyDeadband(flywheelSpeedSupplier.getAsDouble(), 0.2) * TurretConstants.maxFlywheelSpeedRadPerSec;
            
            target.azimuthAngleRad -= MathUtil.applyDeadband(azimuthSpeedSupplier.getAsDouble(), 0.2) * Math.PI * 0.02;
            target.azimuthAngleRad %= Math.PI * 2;

            target.hoodAngleRad -= hoodSpeedSupplier.getAsDouble() * Math.PI * 0.008;
            target.hoodAngleRad = MathUtil.clamp(target.hoodAngleRad, -0.15, 0.9);
        }, () -> {
            target = null;
        }, this);
    }

    public Command runTuning() {
        if(!(io instanceof TurretIOReal)) {
            // TODO: This is really temporary sob
            return Commands.none();
        }
        
        TurretTuning tuning = new TurretTuning((TurretIOReal)io, Controls.getInstance().coDriver);
        return Commands.runOnce(tuning::start).andThen(Commands.runEnd(tuning::run, tuning::stop, this));
    }
}