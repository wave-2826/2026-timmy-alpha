package frc.robot.util.tunables;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunableSimpleMotorFF {
    private SimpleMotorFeedforward ff = null;
    
    public LoggedTunableNumber kS;
    public LoggedTunableNumber kV;
    public LoggedTunableNumber kA;
    
    /** The path to the tunable constants. */
    private String tunablePath;

    /**
     * Creates a new TunableSimpleMotorFF object with the given path. The path is used to create the tunable numbers for the
     * PID constants. Note that this object will never be cleaned up once created, so it should be a constant.
     *
     * @param tunablePath The path to the tunable constants.
     */
    public TunableSimpleMotorFF(String tunablePath) {
        this.tunablePath = tunablePath;
    }

    /** Adds the given gains as defaults to be tuned. */
    public TunableSimpleMotorFF addGains(double kS, double kV, double kA) {
        this.kS = new LoggedTunableNumber(tunablePath + "_kS", kS);
        this.kV = new LoggedTunableNumber(tunablePath + "_kV", kV);
        this.kA = new LoggedTunableNumber(tunablePath + "_kA", kA);
        return this;
    }

    /**
     * Calculates the feedforward from the gains and setpoints assuming discrete control.
     *
     * <p>Note this method is inaccurate when the velocity crosses 0.
     *
     * @param currentVelocity The current velocity setpoint.
     * @param nextVelocity The next velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
        if(this.ff == null || kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
            this.ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
        }

        return ff.calculateWithVelocities(currentVelocity, nextVelocity);
    }
}
