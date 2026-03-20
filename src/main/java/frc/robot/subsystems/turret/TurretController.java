package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.robot.Constants;
import frc.robot.generated.TurretTuningData;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.subsystems.turret.TurretIO.TurretLQROutputs;
import frc.robot.util.tunables.LoggedTunableNumber;
import frc.robot.util.tunables.LoggedTunableVector;

/**
 * The problem formulation (I already made this writeup to talk about it, so I guess it can go in the code):
 * 
 * Our goal is to create a controller for the turret system, which has relatively complex dynamics.
 * 
 * The naive approach is to use PID controllers, but the system has strange coupling and dynamics that
 * make PIDs both hard to tune, error-prone (possible to fall out of the state-space), and often very
 * suboptimal. Therefore, this makes it a MIMO (multiple-input, multiple-output) control problem.
 * 
 * There are a variety of constraints on how the controller should work:
 * - We're operating in a very resource-constrained environment, so it must be fast and consistent.
 *   Though we did testing with them, this rules out controller types like MPCs that rely on solvers.
 * - The code should be easy to implement and understand. Ideally, there's no "black box" like a solver.
 * - We don't need absolutely optimal solutions, but mechanical constraints (such as not allowing the
 *   hood to fall outside of its mechanical limits) and electrical constraints (maximum motor currents)
 *   need to be honored.
 * 
 * Since our control loop runs at 50hz and the system is reasonable enough to linearize around timesteps
 * (...I think), an iterative controller is acceptable. We can control motors around any control loop
 * type, but current control feels most intuitive. Furthermore, we're able to estimate future loads on
 * the system (balls being fed through the turret), so a predictive controller could be potentially
 * beneficial.
 *
 * We already have empirical models of system as three functions I(v_f, v_h, v_a) that take the flywheel,
 * hood, and azimuth velocities and return the current required to keep the turret at a steady state for
 * each motor. These act as the feedforwards for the control model.
 * 
 * There are two primary approaches we considered:
 * - A regular LQR controller, using a linearized state-space model for coupling, applied as a feedback
 *   layer added to the empirical current models. 
 * - A Computed Torque Control (sometimes referred to as Inverse Dynamics Control, which I think means
 *   the same thing?) model that generates and follows motion profiles for each motor.
 * 
 * Regardless of the controller type, our state-space model is represented by the following state vector:
 * x = [
 *     θ_azimuth
 *     ​θ'_azimuth
 *     ​θ_hood
 *     ​θ'_hood
 *     ​ω'_flywheel​​​
 * ]
 * And our input vector is the motor currents:
 * u = [
 *    ​I_azimuth
 *    ​I_hood
 *    ​I_flywheel​​​
 * ]
 * 
 * # The plant
 * 
 * Torque at motor shaft:  τ = Kt * I
 * Angular acceleration:   α = τ / J_reflected  =  Kt * I / J
 * So each motor contributes a_i += (Kt / J) * u_i
 *
 * The continuous-time A matrix captures the kinematics only (position += velocity):
 *
 *         θ_a   θ'_a   θ_h   θ'_h   ω_f
 *  θ_a  [  0     1      0     0      0  ]
 *  θ'_a [  0     0      0     0      0  ]
 *  θ_h  [  0     0      0     1      0  ]
 *  θ'_h [  0     0      0     0      0  ]
 *  ω_f  [  0     0      0     0      0  ]
 *
 * The B matrix maps input currents to state derivatives:
 *         I_azi            I_hood             I_fly
 *  θ_a  [   0                0                  0      ]
 *  θ'_a [ Kt_azi/J_azi       0                  0      ]
 *  θ_h  [   0                0                  0      ]
 *  θ'_h [   0            Kt_hood/J_hood         0      ]
 *  ω_f  [   0                0            Kt_fly/J_fly ]
 *
 * All values are motor-side (since the tuned current models also operate on motor-side velocities).
 *
 * The empirical current models give us the feedforward: the steady-state current needed to
 * hold a given operating point. The LQR then adds a correction current to drive the state
 * error to zero.
 * 
 * Latency compensation is done via LinearSystemLoop.latencyCompensate(), which projects the
 * Kalman observer's state estimate forward in time to account for sensor / CAN lag
 */
public class TurretController {
    // 5 states:  [θ_azi, θ'_azi, θ_hood, θ'_hood, ω'_fly]
    // 3 inputs:  [I_azi, I_hood, I_fly]
    // 5 outputs: full-state measurement (we observe the entire state vector)

    // LQR / Kalman filter tuning

    // State cost (max-error tolerances in the form 1/qMax^2)
    private static final LoggedTunableVector<N5> qWeights = new LoggedTunableVector<>(
        "Turret/LQR_Q",
        VecBuilder.fill(
            Constants.isSim ? 0.002 : 0.0005, // azimuth - rad
            Constants.isSim ? 0.105 : 0.105 , // azimuth vel - rad/s
            Constants.isSim ? 0.005 : 0.005 , // hood - rad
            Constants.isSim ? 0.105 : 0.105 , // hood vel - rad/s
            Constants.isSim ? 2     : 50      // flywheel vel - rad/s
        )
    );
    // R cost weights (control effort tolerance). Decrease this to more heavily penalize
    // control effort, or make the controller less aggressive
    private static final LoggedTunableVector<N3> rWeights = new LoggedTunableVector<>(
        "Turret/LQR_R",
        VecBuilder.fill(
            Constants.isSim ? 50 : 5,
            Constants.isSim ? 50 : 5,
            Constants.isSim ? 50 : 5
        )
    );
    // Kalman process / measurement noise (should be tuned empirically)
    // deg and deg/sec
    private static final Vector<N5> stateStdDevs = VecBuilder.fill(10, 10, 10, 10, 50).times(Math.PI * 2 / 360);
    private static final Vector<N5> measureStdDevs = VecBuilder.fill(0.1, 0.1, 1.0, 0.1, 1.0).times(Math.PI * 2 / 360);
    // Latency compensation
    private static final LoggedTunableNumber lqrLatencyCompSec = new LoggedTunableNumber("Turret/LQR_LatencyComp", Constants.isSim ? 0.0 : 0.05);
    private static final LoggedTunableNumber lqrFFContribution = new LoggedTunableNumber("Turret/LQRFFContribution", Constants.isSim ? 1.0 : 0.1);

    private static final double loopPeriod = 0.02;

    // Kt for a single motor in the group, at motor shaft
    private static final double KtFly = TurretConstants.flywheelSimMotor.KtNMPerAmp;
    private static final double KtAzimuth = TurretConstants.azimuthSimMotor.KtNMPerAmp;
    private static final double KtHood = TurretConstants.hoodSimMotor.KtNMPerAmp;

    // Current-to-acceleration gain for each axis (rad/s^2 per amp, at motor shaft)
    // For the flywheel we have 2 motors sharing the load, so effective J is halved per motor.
    // But TurretConstants.flywheelMotorInertiaKgM2 is already the total reflected inertia seen
    // by one motor (torques add). So, we use the full J and the combined Kt
    private static final double BFlywheel = (KtFly * 2) / TurretConstants.flywheelMotorInertiaKgM2;
    private static final double BAzimuth = KtAzimuth / TurretConstants.azimuthMotorInertiaKgM2;
    private static final double BHood = KtHood / TurretConstants.hoodMotorInertiaKgM2;

    // State-space objects
    private LinearSystem<N5, N3, N5> plant;
    private KalmanFilter<N5, N3, N5> observer;
    private LinearQuadraticRegulator<N5, N3, N5> lqr;
    private LinearSystemLoop<N5, N3, N5> loop;

    public TurretController() {
        // θ_a  θ'_a   θ_h  θ'_h   ω_f
        var A = new Matrix<>(Nat.N5(), Nat.N5());
        A.set(0, 1, 1.0); // θ_azi  += θ'_azi
        A.set(2, 3, 1.0); // θ_hood += θ'_hood
        // All velocity rows are zero in A; dynamics driven entirely by B*u + feedforward

        var B = new Matrix<>(Nat.N5(), Nat.N3());

        // Rows: [θ_a, θ'_a, θ_h, θ'_h, ω_f]
        // Columns: [I_azi, I_hood, I_fly]
        // θ'_azi  accelerated by azimuth current
        B.set(1, 0, BAzimuth * TurretConstants.totalAzimuthGearing); 
        // Coaxial coupling: azimuth current also accelerates hood
        B.set(3, 0, BAzimuth * TurretConstants.azimuthHoodCoupling);
        // Coaxial coupling: azimuth current also accelerates flywheel
        B.set(4, 0, BAzimuth * TurretConstants.azimuthFlyCoupling);

        // θ'_hood accelerated by hood current
        B.set(3, 1, BHood * TurretConstants.totalHoodGearing);
        // ω'_fly  accelerated by flywheel current
        // Negated because we define positive flywheel velocity = shooting direction,
        // which is opposite to the motor direction through the negative gear ratio.
        B.set(4, 2, BFlywheel * -TurretConstants.totalFlywheelGearing);

        var C = Matrix.eye(Nat.N5()); // Just output the full state
        var D = new Matrix<>(Nat.N5(), Nat.N3()); // No direct feedthrough

        plant = new LinearSystem<>(A, B, C, D);

        constructSystem();
    }

    void constructSystem() {
        // ------ Kalman filter ------
        observer = new KalmanFilter<>(Nat.N5(), Nat.N5(), plant, stateStdDevs, measureStdDevs, loopPeriod);

        // ------ LQR ------
        lqr = new LinearQuadraticRegulator<>(plant, qWeights.get(), rWeights.get(), loopPeriod);

        lqr.latencyCompensate(plant, loopPeriod, lqrLatencyCompSec.get());

        loop = new LinearSystemLoop<>(
            plant,
            lqr,
            observer,
            u -> desaturateInputVector(u, VecBuilder.fill(
                TurretConstants.azimuthCurrentLimit * 0.9,
                TurretConstants.hoodCurrentLimit * 0.9,
                TurretConstants.flywheelCurrentLimit * 0.9
            )),
            loopPeriod
        );
    }

    /**
     * Reset the observer the system's measured state
     */
    public void reset(TurretIOInputs inputs) {
        loop.reset(createStateVectorFromInputs(inputs));
    }

    private static Vector<N5> createStateVectorFromInputs(TurretIOInputs inputs) {
        return VecBuilder.fill(
            inputs.getAzimuthAngleRad(),
            inputs.getAzimuthVelocityRadPerSec(),
            inputs.getHoodAngleRad(),
            inputs.getHoodVelocityRadPerSec(),
            inputs.getFlywheelVelocityRadPerSecond()
        );
    }

    /**
     * Run one iteration of the turret LQR control loop.
     *
     * This will:
     * - Build a measurement vector from the current measured inputs  
     * - Calculate the feedforward (steady-state) current for each axis using our empirically-tuned models
     * - Set the next reference target in the LQR loop  
     * - Correct the Kalman observer with the latest measurements  
     * - Compute the LQR correction current and adds it to the feedforward  
     * - Project the observer forward one step  
     *
     * @param inputs  Latest sensor readings from the turret.
     * @param target  Desired setpoint.
     * @return        Motor current commands for each axis.
     */
    public TurretLQROutputs calculate(TurretIOInputs inputs, Turret.TurretTarget target) {
        if(
            qWeights.hasChanged(hashCode()) ||
            rWeights.hasChanged(hashCode()) ||
            lqrLatencyCompSec.hasChanged(hashCode())
        ) constructSystem();

        // Get motor-shaft velocities instead
        double flyMotorVel  = inputs.topFlywheel.velocityRadPerSec();
        double aziMotorVel  = inputs.azimuth.internalEncoderVelocity();
        double hoodMotorVel = inputs.hood.velocityRadPerSec();

        // Empirical feedforward currents
        // The models return steady-state current for the given operating point
        double ffAzimuth  = AzimuthFF(flyMotorVel, aziMotorVel, hoodMotorVel) * lqrFFContribution.get();
        double ffHood     = HoodFF(flyMotorVel, aziMotorVel, hoodMotorVel) * lqrFFContribution.get();
        double ffFlywheel = FlywheelFF(flyMotorVel, aziMotorVel, hoodMotorVel) * lqrFFContribution.get();

        // Build reference vector
        // x_ref = [θ_azi_target, 0 vel, θ_hood_target, 0 vel, ω_fly_target]
        // We command zero velocity at the target (to hold positions / velocity setpoint)
        var reference = VecBuilder.fill(
            target.azimuthAngleRad, 0.0,
            target.hoodAngleRad, 0.0,
            target.flywheelSpeedRadPerSec
        );
        loop.setNextR(reference);

        // Correct the observer with current measurements
        var measurement = createStateVectorFromInputs(inputs);
        observer.correct(loop.getU(), measurement);

        var xHat = observer.getXhat();
        Logger.recordOutput("Turret/LQRKalman/azimuthPosition", xHat.get(0, 0));
        Logger.recordOutput("Turret/LQRKalman/azimuthVelocity", xHat.get(1, 0));
        Logger.recordOutput("Turret/LQRKalman/hoodPosition", xHat.get(2, 0));
        Logger.recordOutput("Turret/LQRKalman/hoodVelocity", xHat.get(3, 0));
        Logger.recordOutput("Turret/LQRKalman/flywheelVelocity", xHat.get(4, 0));
        
        // LQR correction: u = K * error  (feedback only, no plant feedforward)
        // The empirical FF models already capture the steady-state operating-point
        // current, so adding the plant's linear feedforward on top would double-count.
        var error = loop.getNextR().minus(xHat);
        error.set(0, 0, MathUtil.angleModulus(error.get(0, 0))); // Wrap azimuth angle error
        var lqrCorrection = lqr.getK().times(error);

        // Combine: empirical feedforward + LQR correction
        var uOutput = VecBuilder.fill(
            ffAzimuth  + lqrCorrection.get(0, 0),
            ffHood     + lqrCorrection.get(1, 0),
            ffFlywheel + lqrCorrection.get(2, 0)
        );

        var currents = loop.clampInput(uOutput);

        // Predict observer forward using only the LQR correction (the part the
        // plant model knows about). The empirical FF compensates for unmodeled
        // dynamics (friction, coupling losses) and would cause the observer to
        // predict phantom acceleration if included.
        observer.predict(loop.clampInput(lqrCorrection), loopPeriod);

        return new TurretLQROutputs(
            currents.get(2, 0), // flywheel current
            currents.get(0, 0), // azimuth current
            currents.get(1, 0)  // hood current
        );
    }

    /**
     * Feedforward for the azimuth motor: steady-state current at the given operating point.
     * We normalise inputs to positive velocity, query the model, then re-apply the sign.
     */
    public static double AzimuthFF(double flyMotorVel, double aziMotorVel, double hoodMotorVel) {
        double sign = Math.tanh(10 * aziMotorVel);
        if(sign == 0.0) return 0.0;
        return TurretTuningData.AzimuthCurrentModel.calculate(
            flyMotorVel * sign,
            aziMotorVel * sign,
            hoodMotorVel * sign
        );
    }

    /**
     * Feedforward for the hood motor: steady-state current at the given operating point.
     */
    public static double HoodFF(double flyMotorVel, double aziMotorVel, double hoodMotorVel) {
        double sign = Math.tanh(10 * hoodMotorVel);
        if(sign == 0.0) return 0.0;
        return TurretTuningData.HoodCurrentModel.calculate(
            flyMotorVel * sign,
            aziMotorVel * sign,
            hoodMotorVel * sign
        ) * sign;
    }

    /**
     * Feedforward for the flywheel motor: steady-state current at the given operating point.
     */
    public static double FlywheelFF(double flyMotorVel, double aziMotorVel, double hoodMotorVel) {
        double sign = Math.tanh(10 * flyMotorVel);
        if(sign == 0.0) return 0.0;
        return TurretTuningData.FlywheelCurrentModel.calculate(
            flyMotorVel * sign,
            aziMotorVel * sign,
            hoodMotorVel * sign
        ) * sign;
    }

  /**
   * Renormalize all inputs if any exceed the maximum magnitude.
   * Based on StateSpaceUtil.desaturateInputVector but takes a vector instead.
   *
   * @param u The input vector.
   * @param maxMagnitude The maximum magnitude any input can have.
   * @param <I> Number of inputs.
   * @return The normalized input
   */
    private static <I extends Num> Matrix<I, N1> desaturateInputVector(
        Matrix<I, N1> u,
        Vector<I> maxMagnitde
    ) {
        double scalar = 1.0;
        for(int i = 0; i < u.getNumRows(); i++) {
            double ratio = Math.abs(u.get(i, 0) / maxMagnitde.get(i));
            if(ratio > scalar) scalar = ratio;
        }
        return u.div(scalar);
    }
}

