package frc.robot.subsystems.turret.sim;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.generated.TurretTuningData;
import frc.robot.subsystems.turret.TurretConstants;

public final class TurretSimLinear extends LinearSystemSim<N5, N3, N5> implements TurretSim {
    public TurretSimLinear() {
        super(createTurretSystem());
        reset();
    }

    /**
     * Compute the motor damping for a given motor, reflected through the given gear ratio.  
     * Damping is based on the reverse EMF force, which provides a force opposite the direction
     * of motion that is proportional to velocity. 
     * units: (Nm / Amp) / [(rad/s / Volt) * (Volt / Amp)] = Nm / (rad/s)
     * so this represents torque per unit of angular velocity (Nms/rad)
     * @param motor the motor to compute damping for
     */
    private static double computeMotorDamping(DCMotor motor) {
        return -motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms);
    }

    /**
     * Compute the acceleration proportion at the output for the given motor, ratio, and
     * current proportion. Used for linear friction models.
     */
    private static double computeMotorAccelProportion(
        DCMotor motor,
        double currentProportion,
        double momentOfInertiaKgM2) {
        var torqueAtShaft = motor.KtNMPerAmp * currentProportion; // Nm
        // (Nm * ratio^2) / (kg m^2) = rad/s^2
        return torqueAtShaft / momentOfInertiaKgM2;
    }

    private static double flywheelMOI = TurretConstants.flywheelMotorInertiaKgM2;
    private static double azimuthMOI = TurretConstants.azimuthMotorInertiaKgM2;
    private static double hoodMOI = TurretConstants.hoodMotorInertiaKgM2;

    /**
     * Construct a linear system for the turret flywheel, hood, and azimuth.
     * For more information on linear systems, see https://file.tavsys.net/control/controls-engineering-in-frc.pdf
     * 
     * All angular positions and velocities are represented at the motors themselves.
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
        double flywheelDampingTorque = computeMotorDamping(TurretConstants.flywheelSimMotor);
        double hoodDampingTorque = computeMotorDamping(TurretConstants.hoodSimMotor);
        double azimuthDampingTorque = computeMotorDamping(TurretConstants.azimuthSimMotor);

        // damping acceleration in rad/s/s/(rad/s) = 1/s, as expected
        double flywheelDampingAccel = flywheelDampingTorque / flywheelMOI;
        double hoodDampingAccel = hoodDampingTorque / hoodMOI;
        double azimuthDampingAccel = azimuthDampingTorque / azimuthMOI;

        // Linear friction for each stage
        // The current models are in amps, but we want acceleration per velocity (rad/s^2 / rad/s = 1/s);
        double flywheelFricFromAzimuth = computeMotorAccelProportion(TurretConstants.azimuthSimMotor,  TurretTuningData.FlywheelCurrentModel.azimuth, azimuthMOI);
        double flywheelFricFromHood =    computeMotorAccelProportion(TurretConstants.hoodSimMotor,     TurretTuningData.FlywheelCurrentModel.hood,    hoodMOI);
        double hoodFricFromAzimuth =     computeMotorAccelProportion(TurretConstants.azimuthSimMotor,  TurretTuningData.HoodCurrentModel.azimuth,     azimuthMOI);
        double hoodFricFromFlywheel =    computeMotorAccelProportion(TurretConstants.flywheelSimMotor, TurretTuningData.HoodCurrentModel.flywheel,    flywheelMOI);
        double azimuthFricFromFlywheel = computeMotorAccelProportion(TurretConstants.flywheelSimMotor, TurretTuningData.AzimuthCurrentModel.flywheel, flywheelMOI);
        double azimuthFricFromHood =     computeMotorAccelProportion(TurretConstants.hoodSimMotor,     TurretTuningData.AzimuthCurrentModel.hood,     hoodMOI);

        return new LinearSystem<>(
            // System matrix
            MatBuilder.fill(
                Nat.N5(), Nat.N5(),
                
                // This is a very 2D piece of code. prepare to scroll!
                // spotless trambles in fear when it sees this formatting
                // ⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄⌄ -- flywheel velocity derivative, hood position derivative, hood velocity derivative, azimuth position derivative, azimuth velocity derivative
                // [ flywheel vel -= damp,     0, fly vel += hood fric,     0, fly vel += azimuth fric  ]
                     flywheelDampingAccel,     0, flywheelFricFromHood,     0, flywheelFricFromAzimuth,
                // [ 0,                        0, hood pos += velocity,     0, 0                        ]
                     0,                        0, 1,                        0, 0,
                // [ hood vel += fly fric,     0, hood vel -= damp,         0, hood vel += azimuth fric ]
                     hoodFricFromFlywheel,     0, hoodDampingAccel,         0, hoodFricFromAzimuth,
                // [ 0,                        0, 0,                        0, azimuth pos += velocity  ]
                     0,                        0, 0,                        0, 1,
                // [ azimuth vel += fly fric,  0, azimuth vel += hood fric, 0, azimuth vel -= damp      ]
                     azimuthFricFromFlywheel,  0, azimuthFricFromHood,      0, azimuthDampingAccel
            ),
            
            // Input matrix
            MatBuilder.fill(
                Nat.N5(), Nat.N3(),

            //  ⌄--------------⌄----------⌄-- flywheel torque applied, hood torque applied, azimuth torque applied
                1/flywheelMOI, 0, 0,
                0, 0, 0,
                0, 1/hoodMOI, 0,
                0, 0, 0,
                0, 0, 1/azimuthMOI
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
        var updatedXhat = super.updateX(currentXhat, currentU, dtSeconds);

        // We check for collision after updating xhat
        // This isn't an accurate model since it loses energy that would
        // realistically be transferred to other stages, but it's whatever.
        double hoodPosition = updatedXhat.get(1, 0);
        if(hoodPosition < TurretConstants.hoodMinAngle) {
            updatedXhat.set(1, 0, TurretConstants.hoodMinAngle);
            if(updatedXhat.get(2, 0) < 0) {
                updatedXhat.set(2, 0, 0);
            }
        } else if(hoodPosition > TurretConstants.hoodMaxAngle) {
            updatedXhat.set(1, 0, TurretConstants.hoodMaxAngle);
            if(updatedXhat.get(2, 0) > 0) {
                updatedXhat.set(2, 0, 0);
            }
        }
        
        return updatedXhat;
    }

    public TurretState getState() {
        var outputs = getOutput();
        return new TurretState(
            outputs.get(0, 0), // flywheel velocity
            outputs.get(1, 0), // hood position
            outputs.get(2, 0), // hood velocity
            outputs.get(3, 0), // azimuth position
            outputs.get(4, 0)  // azimuth velocity
        );
    }

    @Override
    public void reset() {
        setState(VecBuilder.fill(
            0, // flywheel velocity
            TurretConstants.hoodMinAngle, // hood position
            0, // hood velocity
            0, // azimuth position
            0  // azimuth velocity
        ));
    }

    /**
     * Iterate the turret simulation by the given time step, and return the current state.
     */
    public TurretState updateAndGetState(double flywheelTorque, double hoodTorque, double azimuthTorque, double dtSeconds) {
        Matrix<N3, N1> u = VecBuilder.fill(flywheelTorque, hoodTorque, azimuthTorque);
        setInput(u);
        update(dtSeconds);
        return getState();
    }
}
