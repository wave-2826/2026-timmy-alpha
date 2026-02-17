package frc.robot.subsystems.turret;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public final class TurretSim extends LinearSystemSim<N6, N3, N6> {
    /**
     * Compute the motor damping for a given motor, reflected through the given gear ratio.  
     * Damping is based on the reverse EMF force, which provides a force opposite the direction
     * of motion that is proportional to velocity. 
     * units: (Nm / Amp) / [(rad/s / Volt) * (Volt / Amp)] = Nm / (rad/s)
     * so this represents torque per unit of angular velocity (Nms/rad)
     * @param motor the motor to compute damping for
     * @param ratio the gear ratio between the motor and the output shaft (output/input)
     */
    private static double computeMotorDamping(DCMotor motor, double ratio) {
        var dampingAtShaft = motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms);
        // damping reflects through the gearbox as the square of the gear ratio
        return dampingAtShaft * Math.pow(ratio, 2);
    }

    /**
     * Approximate friction as a continuous function of velocity.
     * tanh(10x) is close to a sign step function but works better for our purposes calculating friction between stages.
     * it's very close absolutely to 1 outside of [-0.3, 0.3].
     */
    private static double frictionFunction(double velocity) {
        return Math.tanh(10 * velocity);
    }

    private static double flywheelMOI = TurretConstants.flywheelMotorInertiaKgM2;
    private static double azimuthMOI = TurretConstants.azimuthMotorInertiaKgM2;
    private static double hoodMOI = TurretConstants.hoodMotorInertiaKgM2;

    /**
     * Construct a linear system for the turret flywheel, hood, and azimuth.
     * For more information on linear systems, see https://file.tavsys.net/control/controls-engineering-in-frc.pdf
     * 
     * All angular positions and velocities are represented at the end of the power transmission:
     * - Flywheel: at the flywheel itself (after all reductions)
     * - Hood: at the hood rotational axis (after all reductions)
     * - Azimuth: turret rotational axis (on the ring)
     * 
     * Positions and velocities are in rotations and rotations per second, not radians.
     * 
     * State (x): [flywheel position, flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * Input (u): [flywheel torque applied, hood torque applied, azimuth torque applied]
     * Output (y): [flywheel position, flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * 
     * We simulate these together instead of using regular flywheel models because they're heavily impacted by each other.
     * Not just coaxially, but with friction between stages.
     */
    public static LinearSystem<N6, N3, N6> createTurretSystem() {
        // Total gearings; these are a ratio between output and input, so should be less than 1.
        var totalFlywheelGearing = TurretConstants.flywheelToRingReduction * TurretConstants.flywheelPlanetReduction * TurretConstants.flywheelBevelReduction;
        var totalHoodGearing = TurretConstants.hoodToRingReduction * TurretConstants.hoodPlanetReduction * TurretConstants.hoodBevelReduction;
        var totalAzimuthGearing = TurretConstants.azimuthToRingReduction;

        // motor damping in Nm/(rad/s)
        double flywheelDampingForce = computeMotorDamping(TurretConstants.flywheelSimMotor, totalFlywheelGearing);
        double hoodDampingForce = computeMotorDamping(TurretConstants.hoodSimMotor, totalHoodGearing);
        double azimuthDampingForce = computeMotorDamping(TurretConstants.azimuthSimMotor, totalAzimuthGearing);

        // damping acceleration in rad/s/s/(rad/s)
        double flywheelDampingAccel = flywheelDampingForce / flywheelMOI;
        double hoodDampingAccel = hoodDampingForce / hoodMOI;
        double azimuthDampingAccel = azimuthDampingForce / azimuthMOI;

        // Coupling ratios
        double azimuthFlyCoupling = TurretConstants.flywheelToRingReduction * TurretConstants.flywheelBevelReduction;
        double azimuthHoodCoupling = TurretConstants.hoodToRingReduction * TurretConstants.hoodBevelReduction;
        
        return new LinearSystem<>(
            // System matrix
            MatBuilder.fill(
                Nat.N6(), Nat.N6(),
                
                // This is a... very 2D piece of code. prepare to scroll!
                // spotless trambles in fear when it sees this formatting
                // ⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄ -- flywheel position derivative, flywheel velocity derivative, hood position derivative, hood velocity derivative, azimuth position derivative, azimuth velocity derivative
                
                // [ 0,                        flywheel pos += velocity, 0,                        0,                        0,                        fly += k * azimuth vel   ]
                     0,                        1,                        0,                        0,                        0,                        azimuthFlyCoupling,
                // [ 0,                        flywheel vel -= damp,     0,                        0,                        0,                        0                        ]
                     0,                        -flywheelDampingAccel,    0,                        0,                        0,                        0,
                // [ 0,                        0,                        0,                        hood pos += velocity,     0,                        hood += k * azimuth vel  ]
                     0,                        0,                        0,                        1,                        0,                        azimuthHoodCoupling,
                // [ 0,                        0,                        0,                        hood vel -= damp,         0,                        0                        ]
                     0,                        0,                        0,                        -hoodDampingAccel,        0,                        0,
                // [ 0,                        0,                        0,                        0,                        0,                        azimuth pos += velocity  ]
                     0,                        0,                        0,                        0,                        0,                        1,
                // [ 0,                        0,                        0,                        0,                        0,                        azimuth vel -= damp      ]
                     0,                        0,                        0,                        0,                        0,                        -azimuthDampingAccel
            ),
            // Input matrix
            MatBuilder.fill(
                Nat.N6(), Nat.N3(),

            //  ⌄--------------⌄----------⌄-- flywheel torque applied, hood torque applied, azimuth torque applied
                0,             0,         0,
                1/flywheelMOI, 0,         0,
                0,             0,         0,
                0,             1/hoodMOI, 0,
                0,             0,         0,
                0,             0,         1/azimuthMOI
            ),
            // Output matrix (identity - just the state)
            Matrix.eye(Nat.N6()),
            // Feedthrough matrix (zero - no direct feedthrough)
            new Matrix<>(Nat.N6(), Nat.N3())
        );
    }

    public TurretSim() {
        super(createTurretSystem());
    }

    
    /**
     * Update the state of the turret.  
     * This isn't a directly linear system, so we use numerical integration to make it nonlinear.
     *
     * @param currentXhat the current state estimate.
     * @param u the system inputs (voltage).
     * @param dtSeconds the time difference between controller updates.
     */
    @Override
    protected Matrix<N6, N1> updateX(Matrix<N6, N1> currentXhat, Matrix<N3, N1> currentU, double dtSeconds) {
        Matrix<N6, N1> updatedXhat = NumericalIntegration.rkdp(
            (Matrix<N6, N1> x, Matrix<N3, N1> u) -> {
                // standard linear dynamics (Ax + Bu)
                Matrix<N6, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(u));

                double flyVel = x.get(1, 0);
                double hoodVel = x.get(3, 0);
                double aziVel = x.get(5, 0);

                // nonlinear coulomb friction
                // friction needs to be relative to the "scrubbing" surfaces
                // double flyFricTorque = -kS_fly * Math.signum(flyVel - (kFA * aziVel));
                // double hoodFricTorque = -kS_hood * Math.signum(hoodVel - (kHA * aziVel));
                // double aziFricTorque = -kS_azi * Math.signum(aziVel);

                // convert torques to acceleration (torque/moi)
                // add to the velocity derivative rows (1, 3, 5)
                return xdot.plus(VecBuilder.fill(0, flyFricTorque / flywheelMOI, 0, hoodFricTorque / hoodMOI, 0, aziFricTorque / azimuthMOI));
            },
            currentXhat,
            currentU,
            dtSeconds);

        // TODO: hard limits on hood
        // // We check for collision after updating xhat
        // if(wouldHitLowerLimit(updatedXhat.get(0, 0))) return VecBuilder.fill(m_minAngle, 0);
        // if(wouldHitUpperLimit(updatedXhat.get(0, 0))) return VecBuilder.fill(m_maxAngle, 0);
        
        return updatedXhat;
    }
}
