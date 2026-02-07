package frc.robot.subsystems.drive.kinematicConstraints;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * This class' purpose is to prevent us from commanding the drivetrain to do something the robot is incapable of carrying out.  
 * Furthermore, it is used to detect slipping wheels and adjust our odometry contributions to improve accuracy in the case of
 * wheel slip.
 * We use three strategies to ensure this:
 * - 
 */
public class KinematicConstraints {
    private class AccelerationLimiter {
        private double value;
        // private double lastTime;

        public AccelerationLimiter(double value) {
            this.value = value;
        }

        /**
         * @param maxAccelerataion Maximum acceleration in units per second
         * @param target Target value to approach
         * @return
         */
        public double calculate(double maxAccelerataion, double target) {
            // double time = Timer.getFPGATimestamp();
            // double deltaTime = Math.min(time - lastTime, 0.1);
            // lastTime = time;
            double deltaTime = 0.02; // whatever

            value += Math.copySign(Math.min(maxAccelerataion * deltaTime, Math.abs(target - value)), target - value);
            return value;
        }
    }

    AccelerationLimiter chassisSpeedMPS = new AccelerationLimiter(0);
    AccelerationLimiter chassisAngularSpeedRadPerSec = new AccelerationLimiter(0);
    AccelerationLimiter tiltXAccelLimiter = new AccelerationLimiter(0);
    AccelerationLimiter tiltYAccelLimiter = new AccelerationLimiter(0);

    LinearAcceleration maxLinearAcceleration;
    AngularAcceleration maxAngularAcceleration;
    LinearAcceleration skidAccelerationLimit;
    LinearAcceleration maxTiltAccelerationX;
    LinearAcceleration maxTiltAccelerationY;

    /**
     * Robot-wide kinematic constraints.
     * 
     * Forward limits:
     *   Forward limits handle the maximum robot (full-system) acceleration based on motor capabilities.  
     *   Because motor torque scales inversely***** with velocity, we can approximate this as a "saturation"
     *   that scales with the ratio of current speed to free speed.
     *   
     *   ***** so many astrisks here... [dynometer data](https://motors.ctr-electronics.com/dyno/dynometer-testing/) is slightly more
     *   accurate, but it doens't account for current limiting, which dramatically changes the curve at lower torques.  
     *   However, based on how we empirically measure maximum acceleration, this "saturation" is a reasonable enough approximation.
     * 
     * Skid limits:
     *   Skid limits are a simple threshold on acceleration that we determine empirically based on how much acceleration
     *   causes the robot wheels to slip on carpet.
     * 
     * Tilt limits:
     *   Tilt limits are per-axis acceleration limtis to prevent the robot from tipping over when accelerating too quickly.
     * 
     * @param maxLinearAcceleration
     * @param maxAngularAcceleration
     * @param skidAccelerationLimit
     * @param maxTiltAccelerationX
     * @param maxTiltAccelerationY
     */
    public KinematicConstraints(
        LinearAcceleration maxLinearAcceleration,
        AngularAcceleration maxAngularAcceleration,
        LinearAcceleration skidAccelerationLimit,
        LinearAcceleration maxTiltAccelerationX,
        LinearAcceleration maxTiltAccelerationY
    ) {
        this.maxLinearAcceleration = maxLinearAcceleration;
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.skidAccelerationLimit = skidAccelerationLimit;
        this.maxTiltAccelerationX = maxTiltAccelerationX;
        this.maxTiltAccelerationY = maxTiltAccelerationY;
    }
    
    public ChassisSpeeds constrainChassisSpeeds(ChassisSpeeds speeds) {
        // Tilt limits
        double vxMetersPerSecond = tiltXAccelLimiter.calculate(maxTiltAccelerationX.in(MetersPerSecondPerSecond), speeds.vxMetersPerSecond);
        double vyMetersPerSecond = tiltYAccelLimiter.calculate(maxTiltAccelerationY.in(MetersPerSecondPerSecond), speeds.vyMetersPerSecond);
        
        // Forward limits
        // TODO: Account for rotational velocity contribution
        double linearSpeedMPS = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);
        double saturation = linearSpeedMPS / DriveConstants.linearFreeSpeed.in(MetersPerSecond);
        double forwardLimitAccel = maxLinearAcceleration.in(MetersPerSecondPerSecond) * (1 - saturation);

        // Skid limits are just constant
        double limitedLinearSpeed = chassisSpeedMPS.calculate(Math.min(
            forwardLimitAccel, skidAccelerationLimit.in(MetersPerSecondPerSecond)
        ), linearSpeedMPS);
        double limitedAngularSpeed = chassisAngularSpeedRadPerSec.calculate(
            maxAngularAcceleration.in(edu.wpi.first.units.Units.RadiansPerSecondPerSecond) * (1 - saturation),
            speeds.omegaRadiansPerSecond
        );
        
        double scale = linearSpeedMPS > 1e-6 ? limitedLinearSpeed / linearSpeedMPS : 0.0;
        return new ChassisSpeeds(vxMetersPerSecond * scale, vyMetersPerSecond * scale, limitedAngularSpeed);
    }
}
