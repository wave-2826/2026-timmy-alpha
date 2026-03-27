package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.turret.Turret.ControlMode;
import frc.robot.subsystems.turret.Turret.TurretTarget;

public class TurretIOTalonHighFrequency implements TurretIO {
    protected static int frequencyHz = 250;

    protected TurretIOTalonFX io = new TurretIOTalonFX();
    protected Notifier notifier;

    /** True once the LQR observer has been seeded with an initial measurement. */
    private boolean controllerInitialised = false;

    private TurretController controller = new TurretController(1. / frequencyHz);

    private ControlMode controlMode = ControlMode.NONE;
    private TurretTarget currentTarget;
    private TurretIOInputs lastInputs;

    private int loopUpdates;
    private double[] latestKalmanState = new double[5];

    public TurretIOTalonHighFrequency() {
        super();
        BaseStatusSignal.setUpdateFrequencyForAll(
            frequencyHz,
            io.azimuthInternalAngle,
            io.azimuthInternalVelocity,
            io.topFlywheelVelocity,
            io.hoodAngle,
            io.hoodVelocity
        );

        Notifier.setHALThreadPriority(true, 99);
        notifier = new Notifier(this::periodic);
        notifier.startPeriodic(1. / frequencyHz);
        notifier.setName("TurretLQR");
    }

    @Override
    public synchronized void resetAzimuth(Rotation2d position) {
        io.resetAzimuth(position);
    }

    @Override
    public synchronized void resetHoodToBottom() {
        io.resetHoodToBottom();
    }

    @Override
    public synchronized void setControlMode(ControlMode mode) {
        controlMode = mode;
    }

    @Override
    public synchronized void setPIDOutputs(TurretIOPIDOutputs outputs) {
        io.setPIDOutputs(outputs);
    }

    @Override
    public synchronized void setVelocityOutputs(double flywheelVelocityRadPerSec, double azimuthVelocityRadPerSec,
            double hoodVelocityRadPerSec) {
        io.setVelocityOutputs(flywheelVelocityRadPerSec, azimuthVelocityRadPerSec, hoodVelocityRadPerSec);
    }

    @Override
    public synchronized void setTarget(TurretTarget target) {
        currentTarget = target;
    }

    @Override
    public synchronized void updateInputs(TurretIOInputs inputs) {
        io.updateInputs(inputs);
        lastInputs = inputs;

        inputs.loopUpdates = loopUpdates;
        inputs.LQRKalmanState = latestKalmanState;
        loopUpdates = 0;
    }

    @Override
    public synchronized void stop() {
        currentTarget = null;
        controllerInitialised = false;
        io.stop();
    }

    protected synchronized void periodic() {
        if(controlMode != ControlMode.LQR || currentTarget == null) {
            controllerInitialised = false;
            return;
        }

        if(!controllerInitialised) {
            // Seed with the measured system state
            controller.reset(lastInputs);
            controllerInitialised = true;
        }

        io.updateLQRInputs(lastInputs);
        var outputs = controller.calculate(lastInputs, currentTarget);
        io.setLQROutputs(outputs);

        latestKalmanState = controller.getObserverState();

        loopUpdates++;
    }
}
