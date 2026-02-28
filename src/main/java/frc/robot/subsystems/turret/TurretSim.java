package frc.robot.subsystems.turret;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.generated.TurretTuningData;

public final class TurretSim extends LinearSystemSim<N5, N3, N5> {
    public enum TurretSimMode {
        LinearSystem,
        MeasuredDynamics
    };

    private TurretSimMode mode = TurretSimMode.LinearSystem;
    public TurretSim(TurretSimMode mode) {
        super(createTurretSystem());
        this.mode = mode;
    }

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
     * Compute the acceleration proportion at the output for the given motor, ratio, and
     * current proportion. Used for linear friction models.
     */
    private static double computeMotorAccelProportion(
        DCMotor motor,
        double ratio,
        double currentProportion,
        double momentOfInertiaKgM2) {
        var torqueAtShaft = motor.KtNMPerAmp * currentProportion; // Nm
        return torqueAtShaft * Math.pow(ratio, 2) / momentOfInertiaKgM2; // (Nm * ratio^2) / (kg m^2) = rad/s^2
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
     * State (x): [flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * Input (u): [flywheel torque applied, hood torque applied, azimuth torque applied]
     * Output (y): [flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * 
     * We simulate these together instead of using regular flywheel models because they're heavily impacted by each other.
     * Not just coaxially, but with friction between stages.
     */
    public static LinearSystem<N5, N3, N5> createTurretSystem() {
        // motor damping in Nm/(rad/s)
        double flywheelDampingForce = computeMotorDamping(TurretConstants.flywheelSimMotor, TurretConstants.totalFlywheelGearing);
        double hoodDampingForce = computeMotorDamping(TurretConstants.hoodSimMotor, TurretConstants.totalHoodGearing);
        double azimuthDampingForce = computeMotorDamping(TurretConstants.azimuthSimMotor, TurretConstants.totalAzimuthGearing);

        // damping acceleration in rad/s/s/(rad/s)
        double flywheelDampingAccel = flywheelDampingForce / flywheelMOI;
        double hoodDampingAccel = hoodDampingForce / hoodMOI;
        double azimuthDampingAccel = azimuthDampingForce / azimuthMOI;

        // Linear friction for each stage
        // The current models are in amps, but we want acceleration per velocity (rad/s^2 / rad/s = 1/s);
        // So we convert the current by 
        double flywheelFricFromAzimuth = computeMotorAccelProportion(TurretConstants.azimuthSimMotor,  TurretConstants.totalAzimuthGearing,  TurretTuningData.FlywheelCurrentModel.azimuth, azimuthMOI);
        double flywheelFricFromHood =    computeMotorAccelProportion(TurretConstants.hoodSimMotor,     TurretConstants.totalHoodGearing,     TurretTuningData.FlywheelCurrentModel.hood,    hoodMOI);
        double hoodFricFromAzimuth =     computeMotorAccelProportion(TurretConstants.azimuthSimMotor,  TurretConstants.totalAzimuthGearing,  TurretTuningData.HoodCurrentModel.azimuth,     azimuthMOI);
        double hoodFricFromFlywheel =    computeMotorAccelProportion(TurretConstants.flywheelSimMotor, TurretConstants.totalFlywheelGearing, TurretTuningData.HoodCurrentModel.flywheel,    flywheelMOI);
        double azimuthFricFromFlywheel = computeMotorAccelProportion(TurretConstants.flywheelSimMotor, TurretConstants.totalFlywheelGearing, TurretTuningData.AzimuthCurrentModel.flywheel, flywheelMOI);
        double azimuthFricFromHood =     computeMotorAccelProportion(TurretConstants.hoodSimMotor,     TurretConstants.totalHoodGearing,     TurretTuningData.AzimuthCurrentModel.hood,     hoodMOI);

        return new LinearSystem<>(
            // System matrix
            MatBuilder.fill(
                Nat.N5(), Nat.N5(),
                
                // This is a very 2D piece of code. prepare to scroll!
                // spotless trambles in fear when it sees this formatting
                // ⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄ -- flywheel velocity derivative, hood position derivative, hood velocity derivative, azimuth position derivative, azimuth velocity derivative
                // [ flywheel pos += velocity, 0, 0,                        0, fly += k * azimuth vel   ]
                // [ flywheel vel -= damp,     0, fly vel += hood fric,     0, fly vel += azimuth fric  ]
                     -flywheelDampingAccel,    0, flywheelFricFromHood,     0, flywheelFricFromAzimuth,
                // [ 0,                        0, hood pos += velocity,     0, 0                        ]
                     0,                        0, 1,                        0, 0,
                // [ hood vel += fly fric,     0, hood vel -= damp,         0, hood vel += azimuth fric ]
                     hoodFricFromFlywheel,     0, -hoodDampingAccel,        0, hoodFricFromAzimuth,
                // [ 0,                        0, 0,                        0, azimuth pos += velocity  ]
                     0,                        0, 0,                        0, 1,
                // [ azimuth vel += fly fric,  0, azimuth vel += hood fric, 0, azimuth vel -= damp      ]
                     azimuthFricFromFlywheel,  0, azimuthFricFromHood,      0, -azimuthDampingAccel
            ),
            // Input matrix
            MatBuilder.fill(
                Nat.N5(), Nat.N3(),

            //  ⌄--------------⌄----------⌄-- flywheel torque applied, hood torque applied, azimuth torque applied
                1/flywheelMOI * Math.pow(TurretConstants.totalFlywheelGearing, 2), 0, TurretConstants.azimuthFlyCoupling/hoodMOI * Math.pow(TurretConstants.totalHoodGearing, 2),
                0, 0, 0,
                0, 1/hoodMOI * Math.pow(TurretConstants.totalHoodGearing, 2), TurretConstants.azimuthHoodCoupling/hoodMOI * Math.pow(TurretConstants.totalHoodGearing, 2),
                0, 0, 0,
                0, 0, 1/azimuthMOI * Math.pow(TurretConstants.totalAzimuthGearing, 2)
            ),
            // Output matrix (identity - just the state)
            Matrix.eye(Nat.N5()),
            // Feedthrough matrix (zero - no direct feedthrough)
            new Matrix<>(Nat.N5(), Nat.N3())
        );
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
    protected Matrix<N5, N1> updateX(Matrix<N5, N1> currentXhat, Matrix<N3, N1> currentU, double dtSeconds) {
        Matrix<N5, N1> updatedXhat = NumericalIntegration.rkdp(
            (Matrix<N5, N1> x, Matrix<N3, N1> u) -> switch (mode) {
                case LinearSystem -> m_plant.getA().times(x).plus(m_plant.getB().times(u));
                case MeasuredDynamics -> TurretController.mcpDynamicsButNumbers(x.getData(), u.getData());
            },
            currentXhat,
            currentU,
            dtSeconds
        );

        // We check for collision after updating xhat
        // This isn't an accurate model since it loses energy that would
        // realistically be transferred to other stages, but it's whatever.
        double hoodPosition = updatedXhat.get(2, 0);
        if(hoodPosition < TurretConstants.hoodMinAngle) {
            updatedXhat.set(2, 0, TurretConstants.hoodMinAngle);
            updatedXhat.set(3, 0, 0);
        } else if(hoodPosition > TurretConstants.hoodMaxAngle) {
            updatedXhat.set(2, 0, TurretConstants.hoodMaxAngle);
            updatedXhat.set(3, 0, 0);
        }
        
        return updatedXhat;
    }

    public class TurretState {
        public double flywheelVelocityRps;
        public double hoodPositionRotations;
        public double hoodVelocityRps;
        public double azimuthPositionRotations;
        public double azimuthVelocityRps;

        public TurretState(Matrix<N5, N1> xhat) {
            flywheelVelocityRps = xhat.get(0, 0);
            hoodPositionRotations = xhat.get(1, 0);
            hoodVelocityRps = xhat.get(2, 0);
            azimuthPositionRotations = xhat.get(3, 0);
            azimuthVelocityRps = xhat.get(4, 0);
        }
    }

    /**
     * Iterate the turret simulation by the given time step, and return the current state.
     */
    public TurretState updateAndGetState(double flywheelTorque, double hoodTorque, double azimuthTorque, double dtSeconds) {
        Matrix<N3, N1> u = VecBuilder.fill(flywheelTorque, hoodTorque, azimuthTorque);
        setInput(u);
        update(dtSeconds);
        return new TurretState(getOutput());
    }
}
