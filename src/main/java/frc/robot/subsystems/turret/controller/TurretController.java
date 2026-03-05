package frc.robot.subsystems.turret.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TurretTuningData;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.util.LoggedTracer;

public class TurretController {
    Matrix<N5, N1> modelState;

    /**
     * The kalman filter observer for the turret state. 
     * The state vector (x) is as follows:
     *     [flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * And the input vector (u) is as follows:
     *     [flywheel current, hood current, azimuth current]ᵀ
     * The output vector is the same as the state vector.
     */
    private ExtendedKalmanFilter<N5, N3, N5> observer = new ExtendedKalmanFilter<N5, N3, N5>(
        Nat.N5(), Nat.N3(), Nat.N5(),
        (x, u) -> numericMpcDynamics(x.getData(), u.getData()),
        (x, u) -> x, // Just output the state vector
        // Model standard deviations
        VecBuilder.fill(
            Units.rotationsPerMinuteToRadiansPerSecond(10),
            Units.degreesToRadians(0.1), Units.rotationsPerMinuteToRadiansPerSecond(0.1),
            Units.degreesToRadians(0.1), Units.rotationsPerMinuteToRadiansPerSecond(0.1)
        ),
        // Measurement standard deviations
        VecBuilder.fill(
            Units.rotationsPerMinuteToRadiansPerSecond(100),
            Units.degreesToRadians(1), Units.rotationsPerMinuteToRadiansPerSecond(1),
            Units.degreesToRadians(1), Units.rotationsPerMinuteToRadiansPerSecond(1)
        ),
        0.02
    );

    private TurretIOInputs inputs;

    public TurretController(TurretIOInputs inputs) {
        this.inputs = inputs;
        modelState = getCurrentState();
        
        // linear system matrices (Ax + Bu)
        // A describes how the state evolves naturally (pos += velocity)
        // This doesn't include reverse EMF deceleration or friction because
        // those are modeled as part of the feedforward current.
        // Matrix<N5, N5> matA = TurretSimLinear.createTurretSystem().getA();
        Matrix<N5, N5> matA = new Matrix<>(Nat.N5(), Nat.N5());
        matA.set(1, 2, 1); // hood pos += vel
        matA.set(3, 4, 1); // azimuth pos += vel

        // B describes how current deltas affect acceleration
        Matrix<N5, N3> matB = new Matrix<>(Nat.N5(), Nat.N3());
        // accel = current * Kt / Inertia
        double fwB = TurretConstants.flywheelSimMotor.KtNMPerAmp / TurretConstants.flywheelMotorInertiaKgM2;
        double hoodB = TurretConstants.hoodSimMotor.KtNMPerAmp / TurretConstants.hoodMotorInertiaKgM2;
        double azB = TurretConstants.azimuthSimMotor.KtNMPerAmp / TurretConstants.azimuthMotorInertiaKgM2;
        
        matB.set(0, 0, fwB);
        matB.set(2, 1, hoodB);
        matB.set(4, 2, azB);
    }

    private Matrix<N5, N1> getCurrentState() {
        return VecBuilder.fill(
            inputs.topFlywheel.flywheelVelocityRadPerSec(),
            MathUtil.clamp(
                inputs.hood.hoodRingAngleRad() / TurretConstants.hoodToRingReduction + TurretConstants.hoodMinAngle,
                TurretConstants.hoodMinAngle,
                TurretConstants.hoodMaxAngle
            ),
            inputs.hood.hoodRingVelocityRadPerSec() / TurretConstants.hoodToRingReduction,
            inputs.azimuth.azimuthAngleRad() / TurretConstants.azimuthToRingReduction,
            inputs.azimuth.azimuthVelocityRadPerSec() / TurretConstants.azimuthToRingReduction
        );
    }

    public double[] getOutputs(
        double targetAzimuthAngle, double targetHoodAngle, double targetFlywheelSpeed
    ) {
        LoggedTracer.skipEpoch();
        
        var reference = new double[] { targetFlywheelSpeed, targetHoodAngle, 0.0, targetAzimuthAngle, 0.0 };
        double[] xHat = observer.getXhat().getData();
        
        double[] uMpc = new double[] { 0.0, 0.0, 0.0 };
        double alpha = 1000.0; // Learning rate
        double h = 1e-4; // Step for finite difference
        double dt = 0.02;
        
        double[] Q = new double[] { 1.0, 10.0, 1.0, 10.0, 1.0 }; // Weights for [fw_vel, hood_pos, hood_vel, az_pos, az_vel]

        for (int iter = 0; iter < 20; iter++) {
            double baseCost = calculateCost(xHat, uMpc, reference, Q, dt);
            double[] grad = new double[3];
            
            for (int i = 0; i < 3; i++) {
                double[] uPlus = uMpc.clone();
                uPlus[i] += h;
                double costPlus = calculateCost(xHat, uPlus, reference, Q, dt);
                grad[i] = (costPlus - baseCost) / h;
            }
            
            for (int i = 0; i < 3; i++) {
                uMpc[i] -= alpha * grad[i];
                double limit, feedforward;

                switch(i) {
                    case 0:
                        feedforward = TurretTuningData.FlywheelCurrentModel.calculate(xHat[0], xHat[2], xHat[4]);
                        limit = TurretConstants.flywheelCurrentLimit;
                        break;
                    case 1:
                        feedforward = TurretTuningData.HoodCurrentModel.calculate(xHat[0], xHat[2], xHat[4]);
                        limit = TurretConstants.hoodCurrentLimit;
                        break;
                    default:
                        feedforward = TurretTuningData.AzimuthCurrentModel.calculate(xHat[0], xHat[2], xHat[4]);
                        limit = TurretConstants.azimuthCurrentLimit;
                        break;
                }

                // Clamp outputs
                uMpc[i] += feedforward;
                uMpc[i] = MathUtil.clamp(uMpc[i], -limit, limit);
            }
        }

        double[] totalOutput = uMpc;

        // Update kalman filter
        try {
            var inputVec = VecBuilder.fill(totalOutput[0], totalOutput[1], totalOutput[2]);
            observer.predict(inputVec, dt);
            observer.correct(inputVec, getCurrentState());
        } catch(Exception e) {
            e.printStackTrace();
        }

        LoggedTracer.record("Turret/SolveEnd");
        return totalOutput;
    }

    private double calculateCost(double[] state, double[] input, double[] reference, double[] Q, double dt) {
        Matrix<N5, N1> dxdt = numericMpcDynamics(state, input);
        double cost = 0.0;
        for (int i = 0; i < 5; i++) {
            double nextStateVal = state[i] + dxdt.get(i, 0) * dt;
            double error = nextStateVal - reference[i];
            cost += error * error * Q[i];
        }
        return cost;
    }

    public static Matrix<N5, N1> numericMpcDynamics(double[] state, double[] input) {
        double azimuthMotorVelocity = state[4];
        double flywheelMotorVelocity = state[0];
        double hoodMotorVelocity = state[2];

        double flywheelSteadyStateCurrent = TurretTuningData.FlywheelCurrentModel.calculate(flywheelMotorVelocity, hoodMotorVelocity, azimuthMotorVelocity);
        double hoodSteadyStateCurrent = TurretTuningData.HoodCurrentModel.calculate(flywheelMotorVelocity, hoodMotorVelocity, azimuthMotorVelocity);
        double azimuthSteadyStateCurrent = TurretTuningData.AzimuthCurrentModel.calculate(flywheelMotorVelocity, hoodMotorVelocity, azimuthMotorVelocity);

        double flywheelCurrent = input[0] - flywheelSteadyStateCurrent;
        double hoodCurrent = input[1] - hoodSteadyStateCurrent;
        double azimuthCurrent = input[2] - azimuthSteadyStateCurrent;

        double flywheelMotorAcceleration = flywheelCurrent * TurretConstants.flywheelSimMotor.KtNMPerAmp / TurretConstants.flywheelMotorInertiaKgM2;
        double hoodMotorAcceleration = hoodCurrent * TurretConstants.hoodSimMotor.KtNMPerAmp / TurretConstants.hoodMotorInertiaKgM2;
        double azimuthMotorAcceleration = azimuthCurrent * TurretConstants.azimuthSimMotor.KtNMPerAmp / TurretConstants.azimuthMotorInertiaKgM2;

        return VecBuilder.fill(
            flywheelMotorAcceleration, state[2],
            hoodMotorAcceleration, state[4],
            azimuthMotorAcceleration
        );
    }
}
