package frc.robot.subsystems.turret;

public class TurretSim {
    public record SimTurretState(
        double flywheelMotorVelRps,
        double hoodMotorPosRad,
        double hoodMotorVelRps,
        double azimuthMotorPosRad,
        double azimuthMotorVelRps
    ) {
        /** Get the velocity of the flywheel itself in radians per second */
        public double flywheelVelRps() {
            return flywheelMotorVelRps * TurretConstants.totalFlywheelGearing -
                azimuthMotorVelRps * TurretConstants.azimuthFlyCoupling;
        }
        /** Get the velocity of the hood itself in radians per second */
        public double hoodVelRps() {
            return hoodMotorVelRps * TurretConstants.totalHoodGearing -
                azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling;
        }
        /** Get the velocity of the azimuth itself in radians per second */
        public double azimuthVelRps() {
            return azimuthMotorVelRps * TurretConstants.aziMotorToRingReduction;
        }

        /** Get the hood position in radians */
        public double hoodPosRad() {
            return hoodMotorPosRad * TurretConstants.totalHoodGearing -
                azimuthMotorPosRad * TurretConstants.azimuthHoodCoupling +
                TurretConstants.hoodMinAngle;
        }
        /** Get the azimuth position in radians */
        public double azimuthPosRad() {
            return azimuthMotorPosRad * TurretConstants.aziMotorToRingReduction;
        }
    }

    private double flywheelMotorVelRps = 0;
    private double hoodMotorPosRad = 0;
    private double hoodMotorVelRps = 0;
    private double azimuthMotorPosRad = 0;
    private double azimuthMotorVelRps = 0;

    public void reset() {
        flywheelMotorVelRps = 0;
        hoodMotorPosRad = 0;
        hoodMotorVelRps = 0;
        azimuthMotorPosRad = 0;
        azimuthMotorVelRps = 0;
    }
    
    public SimTurretState getState() {
        return new SimTurretState(
            flywheelMotorVelRps,
            hoodMotorPosRad,
            hoodMotorVelRps,
            azimuthMotorPosRad,
            azimuthMotorVelRps
        );
    }

    // Small Coulomb friction in amps
    private static final double coulombFrictionAmps = 2.0;

    /** Apply Coulomb friction: subtract a fixed opposing current, clamped so it can't reverse. */
    private double applyCoulombFriction(double current, double velocity) {
        if(Math.abs(velocity) < 1e-3) {
            // Static friction: absorb current up to the friction threshold
            if(Math.abs(current) < coulombFrictionAmps) return 0.0;
            return current - Math.copySign(coulombFrictionAmps, current);
        }
        // Kinetic friction: always opposes velocity
        return current - Math.copySign(coulombFrictionAmps, velocity);
    }

    /**
     * Iterate the turret simulation by the given time step, and return the current state.
     */
    public SimTurretState updateAndGetState(double flywheelVoltage, double hoodVoltage, double azimuthVoltage, double dtSeconds) {
        // Calculate the actual current drawn by each motor given the applied voltage and current velocity.
        // DCMotor.getCurrent already accounts for back-EMF, so at free speed the current naturally drops to zero.
        double flywheelCurrent = TurretConstants.flywheelSimMotor.getCurrent(flywheelMotorVelRps, flywheelVoltage);
        double azimuthCurrent = TurretConstants.azimuthSimMotor.getCurrent(azimuthMotorVelRps, azimuthVoltage);
        double hoodCurrent = TurretConstants.hoodSimMotor.getCurrent(hoodMotorVelRps, hoodVoltage);

        // Apply small Coulomb friction
        double flywheelAppliedCurrent = applyCoulombFriction(flywheelCurrent, flywheelMotorVelRps);
        double azimuthAppliedCurrent  = applyCoulombFriction(azimuthCurrent, azimuthMotorVelRps);
        double hoodAppliedCurrent     = applyCoulombFriction(hoodCurrent, hoodMotorVelRps);

        // Update the velocities based on the applied current
        // A * (Nm/A) / (Kg m^2) = rad/s^2
        double flywheelAngularAcceleration = flywheelAppliedCurrent * TurretConstants.flywheelSimMotor.KtNMPerAmp / TurretConstants.flywheelMotorInertiaKgM2;
        double azimuthAngularAcceleration = azimuthAppliedCurrent * TurretConstants.azimuthSimMotor.KtNMPerAmp / TurretConstants.azimuthMotorInertiaKgM2;
        double hoodAngularAcceleration = hoodAppliedCurrent * TurretConstants.hoodSimMotor.KtNMPerAmp / TurretConstants.hoodMotorInertiaKgM2;
        
        flywheelMotorVelRps += flywheelAngularAcceleration * dtSeconds;
        azimuthMotorVelRps += azimuthAngularAcceleration * dtSeconds;
        hoodMotorVelRps += hoodAngularAcceleration * dtSeconds;
        
        azimuthMotorPosRad += azimuthMotorVelRps * dtSeconds;
        hoodMotorPosRad += hoodMotorVelRps * dtSeconds;

        // Hood hard limits
        // hoodPosRad = hoodMotorPosRad * totalHoodGearing - azimuthMotorPosRad * azimuthHoodCoupling + hoodMinAngle
        // When the hood hits a limit we clamp hoodMotorPosRad and zero the hood motor velocity
        // Technically the azimuth should be affected, but it's affected in a non-trivial way so we don't
        // model it. Ideally, the hood would never hit anyway :)
        double hoodPos = getState().hoodPosRad();
        if (hoodPos > TurretConstants.hoodMaxAngle) {
            hoodMotorPosRad = (TurretConstants.hoodMaxAngle - TurretConstants.hoodMinAngle
                + azimuthMotorPosRad * TurretConstants.azimuthHoodCoupling)
                / TurretConstants.totalHoodGearing;
            double hoodMechVel = hoodMotorVelRps * TurretConstants.totalHoodGearing
                - azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling;
            if (hoodMechVel > 0) {
                // Only stall the hood motor, not azimuth
                hoodMotorVelRps = azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling
                    / TurretConstants.totalHoodGearing;
            }
        } else if (hoodPos < TurretConstants.hoodMinAngle) {
            hoodMotorPosRad = azimuthMotorPosRad * TurretConstants.azimuthHoodCoupling
                / TurretConstants.totalHoodGearing;
            double hoodMechVel = hoodMotorVelRps * TurretConstants.totalHoodGearing
                - azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling;
            if (hoodMechVel < 0) {
                hoodMotorVelRps = azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling
                    / TurretConstants.totalHoodGearing;
            }
        }

        return getState();
    }
}
