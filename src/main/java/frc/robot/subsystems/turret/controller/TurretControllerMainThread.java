package frc.robot.subsystems.turret.controller;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.turret.Turret.TurretTarget;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

public class TurretControllerMainThread implements TurretControllerIO {
    public TurretController controller;

    private TurretMPCOutputs latestOutput = new TurretMPCOutputs(0.0, 0.0, 0.0);
    private double lastTime = 0.0;

    @Override
    public void init(TurretIOInputs inputs) {
        controller = new TurretController(inputs);
    }

    @Override
    public void getOutput(TurretControllerIOInputs inputs) {
        inputs.mpc = latestOutput;
        inputs.computationTimeMs = lastTime;
    }

    @Override
    public void run(TurretTarget target) {
        double startTime = Timer.getFPGATimestamp();
        double[] outputs = controller.getOutputs(target.azimuthAngleRad, target.hoodAngleRad, target.flywheelSpeedRadPerSec);
        latestOutput = new TurretMPCOutputs(
            outputs[0], outputs[1], outputs[2]
        );
        lastTime = (Timer.getFPGATimestamp() - startTime) * 1000;
    }
}
