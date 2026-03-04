package frc.robot.subsystems.turret.controller;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.turret.Turret.TurretTarget;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

/**
 * For the controller to run on a separate thread, it needs to be put in an IO
 * implementation since threading can't be re-created during replay.  
 */
public interface TurretControllerIO {
    @AutoLog
    public static class TurretControllerIOInputs {
        public TurretMPCOutputs mpc;
        public double computationTimeMs;
    }

    public static record TurretMPCOutputs(
        /** The current to apply to the flywheel motors (half to each), in amps. */
        double flywheelCurrent,
        /** The current to apply to the hood motor, in amps. */
        double hoodCurrent,
        /** The current to apply to the azimuth motor, in amps. */
        double azimuthCurrent
    ) {}

    public default void init(
        TurretIOInputs inputs
    ) {}

    public default void getOutput(TurretControllerIOInputs inputs) {}
    public default void run(TurretTarget target) {}
}
