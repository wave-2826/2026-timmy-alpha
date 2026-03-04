package frc.robot.subsystems.turret.controller;

import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.Problem;

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
import frc.robot.util.solver.NonlinearMPC;
import frc.robot.util.solver.NonlinearMPC.InitialGuess;

public class TurretController {
    Matrix<N5, N1> modelState;

    /**
     * The MPC controller for our turret.  
     * The state vector (x) is as follows:
     *     [flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * And the input vector (u) is as follows:
     *     [flywheel current, hood current, azimuth current]ᵀ
     * Positions and velocities are relative motors, not mechanisms - therefore, e.g. the hood position isn't
     * proportional to the true hood.  
     * Angles are in radians and angular velocities in rad/s.
     */
    private NonlinearMPC mpc;

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
        (x, u) -> mcpDynamicsButNumbers(x.getData(), u.getData()),
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

        mpc = new NonlinearMPC(
            5, 3,
            this::mpcDynamics,
            0.01,
            this::mpcCost, this::applyMpcConstraints, this::mpcInitialGuess,
            0.1,
            1.0, // Timeout, s
            0.1 // Tolerance, A
        );
    }

    private Matrix<N5, N1> getCurrentState() {
        return VecBuilder.fill(
            inputs.topFlywheel.flywheelVelocityRadPerSec(),
            inputs.hood.hoodRingAngleRad() / TurretConstants.hoodToRingReduction,
            inputs.hood.hoodRingVelocityRadPerSec() / TurretConstants.hoodToRingReduction,
            inputs.azimuth.azimuthAngleRad() / TurretConstants.azimuthToRingReduction,
            inputs.azimuth.azimuthVelocityRadPerSec() / TurretConstants.azimuthToRingReduction
        );
    }

    public double[] getOutputs(
        double targetAzimuthAngle, double targetHoodAngle, double targetFlywheelSpeed
    ) {
        LoggedTracer.skipEpoch();
        
        LoggedTracer.record("Turret/Kalman");
        
        double[] reference = new double[] {
            targetFlywheelSpeed,
            targetHoodAngle,
            targetAzimuthAngle
        };
        var solution = mpc.calculate(observer.getXhat().getData(), reference);

        LoggedTracer.record("Turret/Solution");

        try {
            var input = VecBuilder.fill(solution[0], solution[1], solution[2]);
            observer.predict(input, 0.02);
            observer.correct(input, getCurrentState());
        } catch(Exception e) {
            e.printStackTrace();
        }

        return solution;
    }

    private VariableMatrix mpcDynamics(VariableMatrix state, VariableMatrix input) {
        // The current equations we tune represent the current required to achieve a steady-state condition.
        // Therefore, the "available acceleration current" is the difference between our input and the
        // steady-state current.
        Variable azimuthMotorVelocity = state.get(4, 0);
        Variable flywheelMotorVelocity = state.get(0, 0);
        Variable hoodMotorVelocity = state.get(2, 0);

        Variable flywheelSteadyStateCurrent = TurretTuningData.FlywheelCurrentModel.calculate(
            flywheelMotorVelocity, hoodMotorVelocity, azimuthMotorVelocity
        );
        Variable hoodSteadyStateCurrent = TurretTuningData.HoodCurrentModel.calculate(
            flywheelMotorVelocity, hoodMotorVelocity, azimuthMotorVelocity
        );
        Variable azimuthSteadyStateCurrent = TurretTuningData.AzimuthCurrentModel.calculate(
            flywheelMotorVelocity, hoodMotorVelocity, azimuthMotorVelocity
        );

        Variable flywheelCurrent = input.get(0, 0).minus(flywheelSteadyStateCurrent);
        Variable hoodCurrent = input.get(1, 0).minus(hoodSteadyStateCurrent);
        Variable azimuthCurrent = input.get(2, 0).minus(azimuthSteadyStateCurrent);

        // Current (A) * Torque Constant (Nm/A) / Inertia (kg*m^2) = Acceleration (rad/s^2)
        Variable flywheelMotorAcceleration = flywheelCurrent
            .times(TurretConstants.flywheelSimMotor.KtNMPerAmp / TurretConstants.flywheelMotorInertiaKgM2);
        Variable hoodMotorAcceleration = hoodCurrent
            .times(TurretConstants.hoodSimMotor.KtNMPerAmp / TurretConstants.hoodMotorInertiaKgM2);
        Variable azimuthMotorAcceleration = azimuthCurrent
            .times(TurretConstants.azimuthSimMotor.KtNMPerAmp / TurretConstants.azimuthMotorInertiaKgM2);
        
        return NonlinearMPC.columnMatrix(new Variable[] {
            // d(flywheel velocity)/dt
            flywheelMotorAcceleration,
            // d(hood position)/dt = hood velocity
            state.get(2, 0),
            // d(hood velocity)/dt
            hoodMotorAcceleration,
            // d(azimuth position)/dt = azimuth velocity
            state.get(4, 0),
            // d(azimuth velocity)/dt
            azimuthMotorAcceleration
        });
    }

    public static Matrix<N5, N1> mcpDynamicsButNumbers(double[] state, double[] input) {
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

    private Variable mpcCost(VariableMatrix state, VariableMatrix input, VariableMatrix reference) {
        Variable azimuthMotorVelocity = state.get(4, 0);
        Variable azimuthMotorPosition = state.get(3, 0);
        Variable flywheelVelocity = state.get(0, 0).times(TurretConstants.totalFlywheelGearing)
            .minus(azimuthMotorVelocity.times(TurretConstants.azimuthFlyCoupling));
        Variable hoodPosition = state.get(1, 0).times(TurretConstants.totalHoodGearing)
            .minus(azimuthMotorPosition.times(TurretConstants.azimuthHoodCoupling));
        Variable azimuthPosition = azimuthMotorPosition.times(TurretConstants.totalAzimuthGearing);

        // Cost is basically just tracking error, but we add small penalties for current usage
        // Normalize to avoid overemphasizing flywheel
        Variable flywheelVelocityError = reference.get(0).minus(flywheelVelocity)
            .times(1 / TurretConstants.maxFlywheelSpeedRadPerSec * 0.5);
        Variable hoodPositionError = reference.get(1).minus(hoodPosition);
        Variable azimuthPositionError = reference.get(2).minus(azimuthPosition);

        Variable cost = flywheelVelocityError.times(flywheelVelocityError)
            .plus(hoodPositionError.times(hoodPositionError))
            .plus(azimuthPositionError.times(azimuthPositionError));
        
        for(int i = 0; i < 3; i++) {
            Variable current = input.get(i, 0);
            cost = cost.plus(current.times(current));
        }
        
        return cost;
    }

    private void applyMpcConstraints(Problem problem, VariableMatrix state, VariableMatrix inputs) {
        // Hood position
        var hoodAngle = state.get(1, 0);
        problem.subjectTo(Constraints.bounds(TurretConstants.hoodMinAngle, hoodAngle, TurretConstants.hoodMaxAngle));

        // Maximum current output
        var flywheelCurrent = inputs.get(0, 0);
        var hoodCurrent = inputs.get(1, 0);
        var azimuthCurrent = inputs.get(2, 0);
        problem.subjectTo(Constraints.bounds(-TurretConstants.flywheelCurrentLimit, flywheelCurrent, TurretConstants.flywheelCurrentLimit));
        problem.subjectTo(Constraints.bounds(-TurretConstants.hoodCurrentLimit, hoodCurrent, TurretConstants.hoodCurrentLimit));
        problem.subjectTo(Constraints.bounds(-TurretConstants.azimuthCurrentLimit, azimuthCurrent, TurretConstants.azimuthCurrentLimit));
    }

    private InitialGuess mpcInitialGuess(double[] currentState, double[] reference, int samplesN) {
        return new InitialGuess(getCurrentState().getData(), new double[] {
            0, 0, 0
        });
    }
}
