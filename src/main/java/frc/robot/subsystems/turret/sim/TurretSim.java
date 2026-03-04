package frc.robot.subsystems.turret.sim;

import frc.robot.subsystems.turret.TurretConstants;

public interface TurretSim {
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
            return azimuthMotorVelRps * TurretConstants.azimuthToRingReduction;
        }

        /** Get the hood position in radians */
        public double hoodPosRad() {
            return hoodMotorPosRad * TurretConstants.totalHoodGearing -
                azimuthMotorPosRad * TurretConstants.azimuthHoodCoupling +
                TurretConstants.hoodMinAngle;
        }
        /** Get the azimuth position in radians */
        public double azimuthPosRad() {
            return azimuthMotorPosRad * TurretConstants.azimuthToRingReduction;
        }
    }

    public void reset();
    public TurretState getState();
    /**
     * Iterate the turret simulation by the given time step, and return the current state.
     */
    public TurretState updateAndGetState(double flywheelVoltage, double hoodVoltage, double azimuthVoltage, double dtSeconds);
}
