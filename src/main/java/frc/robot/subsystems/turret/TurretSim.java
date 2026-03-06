package frc.robot.subsystems.turret;

import frc.robot.generated.TurretTuningData;

public class TurretSim {
    public record TurretState(
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
    
    public TurretState getState() {
        return new TurretState(
            flywheelMotorVelRps,
            hoodMotorPosRad,
            hoodMotorVelRps,
            azimuthMotorPosRad,
            azimuthMotorVelRps
        );
    }

    private double decreaseUnsignedMagnitude(double of, double amount) {
        if(of > 0) return Math.max(of - amount, 0);
        else return Math.min(of + amount, 0);
    } 

    /**
     * Iterate the turret simulation by the given time step, and return the current state.
     */
    public TurretState updateAndGetState(double flywheelVoltage, double hoodVoltage, double azimuthVoltage, double dtSeconds) {
        // Calculate the actual current drawn by each motor given the applied voltage and current velocity
        double flywheelCurrent = TurretConstants.flywheelSimMotor.getCurrent(flywheelMotorVelRps, flywheelVoltage);
        double azimuthCurrent = TurretConstants.azimuthSimMotor.getCurrent(azimuthMotorVelRps, azimuthVoltage);
        double hoodCurrent = TurretConstants.hoodSimMotor.getCurrent(hoodMotorVelRps, hoodVoltage);

        // The tuned current models predict total steady-state current consumption (always a large positive
        // magnitude due to the bias term, ~9-12A). They represent frictional/back-EMF losses that always
        // oppose motion.
        // We normalize the inputs to have positive velocity for each motors' model then multiply by the
        // velocity sign to find the appropriate direction
        double flySignFlip = -Math.signum(flywheelMotorVelRps);
        double azimuthSignFlip = -Math.signum(azimuthMotorVelRps);
        double hoodSignFlip = -Math.signum(hoodMotorVelRps);
        double flywheelResistantCurrent = TurretTuningData.FlywheelCurrentModel.calculate(
            flywheelMotorVelRps * flySignFlip,
            azimuthMotorVelRps * flySignFlip,
            hoodMotorVelRps * flySignFlip
        ) * Math.signum(flywheelMotorVelRps);
        double azimuthResistantCurrent = TurretTuningData.AzimuthCurrentModel.calculate(
            flywheelMotorVelRps * azimuthSignFlip,
            azimuthMotorVelRps * azimuthSignFlip,
            hoodMotorVelRps * azimuthSignFlip
        ) * Math.signum(azimuthMotorVelRps);
        double hoodResistantCurrent = TurretTuningData.HoodCurrentModel.calculate(
            flywheelMotorVelRps * hoodSignFlip,
            azimuthMotorVelRps * hoodSignFlip,
            hoodMotorVelRps * hoodSignFlip
        ) * Math.signum(hoodMotorVelRps);

        double flywheelAppliedCurrent = decreaseUnsignedMagnitude(flywheelCurrent, flywheelResistantCurrent);
        double azimuthAppliedCurrent = decreaseUnsignedMagnitude(azimuthCurrent, azimuthResistantCurrent);
        double hoodAppliedCurrent = decreaseUnsignedMagnitude(hoodCurrent, hoodResistantCurrent);

        // Update the velocities based on the applied current
        // A * (Nm/A) / (Kg m^2) = rad/s^2 (2pi isn't needed here for some reason)
        double flywheelAngularAcceleration = flywheelAppliedCurrent *
            TurretConstants.flywheelSimMotor.KtNMPerAmp / TurretConstants.flywheelMotorInertiaKgM2;
        double azimuthAngularAcceleration = azimuthAppliedCurrent *
            TurretConstants.azimuthSimMotor.KtNMPerAmp / TurretConstants.azimuthMotorInertiaKgM2;
        double hoodAngularAcceleration = hoodAppliedCurrent *
            TurretConstants.hoodSimMotor.KtNMPerAmp / TurretConstants.hoodMotorInertiaKgM2;
        
        flywheelMotorVelRps += flywheelAngularAcceleration * dtSeconds;
        azimuthMotorVelRps += azimuthAngularAcceleration * dtSeconds;
        hoodMotorVelRps += hoodAngularAcceleration * dtSeconds;
        
        azimuthMotorPosRad += azimuthMotorVelRps * dtSeconds;
        hoodMotorPosRad += hoodMotorVelRps * dtSeconds;

        // hood hard limits
        // hoodPosRad = hoodMotorPosRad * totalHoodGearing - azimuthMotorPosRad * azimuthHoodCoupling + hoodMinAngle
        // hoodVelRps = hoodMotorVelRps * totalHoodGearing - azimuthMotorVelRps * azimuthHoodCoupling
        //
        // when the hood hits a limit the mechanism velocity must be zero, which means both the hood
        // motor and the azimuth motor (via coupling) are stalled against the hard stop
        // we clamp hoodMotorPosRad to keep hoodPosRad() within [hoodMinAngle, hoodMaxAngle], then
        // zero out whichever velocity components are still trying to drive further into the limit
        double hoodPos = getState().hoodPosRad();
        if(hoodPos > TurretConstants.hoodMaxAngle) {
            // clamp motor position so hoodPosRad() == hoodMaxAngle exactly
            hoodMotorPosRad = (TurretConstants.hoodMaxAngle - TurretConstants.hoodMinAngle
                + azimuthMotorPosRad * TurretConstants.azimuthHoodCoupling)
                / TurretConstants.totalHoodGearing;
            // kill mechanism velocity still pushing toward the limit
            double hoodMechVel = hoodMotorVelRps * TurretConstants.totalHoodGearing
                - azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling;
            if(hoodMechVel > 0) {
                hoodMotorVelRps = 0;
                azimuthMotorVelRps = 0;
            }
        } else if(hoodPos < TurretConstants.hoodMinAngle) {
            // clamp motor position so hoodPosRad() == hoodMinAngle exactly
            hoodMotorPosRad = (azimuthMotorPosRad * TurretConstants.azimuthHoodCoupling)
                / TurretConstants.totalHoodGearing;
            // kill any mechanism velocity still pushing toward the limit
            double hoodMechVel = hoodMotorVelRps * TurretConstants.totalHoodGearing
                - azimuthMotorVelRps * TurretConstants.azimuthHoodCoupling;
            if(hoodMechVel < 0) {
                hoodMotorVelRps = 0;
                azimuthMotorVelRps = 0;
            }
        }

        return getState();
    }
}
