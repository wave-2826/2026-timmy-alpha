package frc.robot.subsystems.turret;

import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.Problem;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TurretTuningData;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
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
     * Positions and velocities are relative to the actuated mechanisms (actual flywheel, hood, and azimuth).  
     * Angles are in radians and angular velocities in rad/s.
     */
    private NonlinearMPC mpc;

    /**
     * The kalman filter observer for the turret state. 
     * The state vector (x) is as follows:
     *     [flywheel velocity, hood position, hood velocity, azimuth position, azimuth velocity]ᵀ
     * And the input vector (u) is as follows:
     *     [...model state vector, flywheel current, hood current, azimuth current]ᵀ
     * The output vector is the same as the state vector.
     */
    private ExtendedKalmanFilter<N5, N5, N5> observer = new ExtendedKalmanFilter<N5, N5, N5>(
        Nat.N5(), Nat.N5(), Nat.N5(),
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
            0.5,
            0.003, 0.001
        );
    }

    private Matrix<N5, N1> getCurrentState() {
        return VecBuilder.fill(
            (inputs.topFlywheel.velocityRadPerSec() + inputs.bottomFlywheel.velocityRadPerSec()) / 2 + inputs.azimuth.azimuthVelocityRadPerSec() * TurretConstants.azimuthFlyCoupling,
            TurretConstants.hoodMinAngle,
            inputs.hood.hoodRingVelocityRadPerSec() - inputs.azimuth.azimuthVelocityRadPerSec() * TurretConstants.azimuthHoodCoupling,
            inputs.azimuth.azimuthAngleRad(),
            inputs.azimuth.azimuthVelocityRadPerSec()
        );
    }

    public double[] getOutputs(
        double targetAzimuthAngle, double targetHoodAngle, double targetFlywheelSpeed
    ) {
        observer.correct(modelState, getCurrentState());

        double[] reference = new double[] {
            targetFlywheelSpeed,
            targetHoodAngle,
            targetAzimuthAngle
        };
        var solution = mpc.calculate(observer.getXhat().getData(), reference);

        return solution;
    }

    private VariableMatrix mpcDynamics(VariableMatrix state, VariableMatrix input) {
        // The current equations we tune represent the current required to achieve a steady-state condition.
        // Therefore, the "available acceleration current" is the difference between our input and the
        // steady-state current.
        Variable flywheelSteadyStateCurrent = TurretTuningData.FlywheelCurrentModel.calculate(
            input.get(0, 0), input.get(1, 0), input.get(2, 0)
        );
        Variable hoodSteadyStateCurrent = TurretTuningData.HoodCurrentModel.calculate(
            input.get(0, 0), input.get(1, 0), input.get(2, 0)
        );
        Variable azimuthSteadyStateCurrent = TurretTuningData.AzimuthCurrentModel.calculate(
            input.get(0, 0), input.get(1, 0), input.get(2, 0)
        );

        Variable flywheelCurrent = input.get(0, 0).minus(flywheelSteadyStateCurrent);
        Variable hoodCurrent = input.get(1, 0).minus(hoodSteadyStateCurrent);
        Variable azimuthCurrent = input.get(2, 0).minus(azimuthSteadyStateCurrent);

        // Current (A) * Torque Constant (Nm/A) / Inertia (kg*m^2) = Acceleration (rad/s^2)
        Variable flywheelMotorAcceleration = flywheelCurrent
            .times(TurretConstants.flywheelSimMotor.KtNMPerAmp)
            .div(TurretConstants.flywheelMotorInertiaKgM2);
        Variable hoodMotorAcceleration = hoodCurrent
            .times(TurretConstants.hoodSimMotor.KtNMPerAmp)
            .div(TurretConstants.hoodMotorInertiaKgM2);
        Variable azimuthMotorAcceleration = azimuthCurrent
            .times(TurretConstants.azimuthSimMotor.KtNMPerAmp)
            .div(TurretConstants.azimuthMotorInertiaKgM2);
        
        // Acceleration at the mechanism itself is reduced by the gear ratio
        Variable azimuthAcceleration = azimuthMotorAcceleration.times(Math.pow(TurretConstants.totalAzimuthGearing, 2));
        
        Variable flywheelAcceleration = flywheelMotorAcceleration
            .times(Math.pow(TurretConstants.totalFlywheelGearing, 2))
            .plus(azimuthAcceleration.times(TurretConstants.azimuthFlyCoupling));
        Variable hoodAcceleration = hoodMotorAcceleration
            .times(Math.pow(TurretConstants.totalHoodGearing, 2))
            .plus(azimuthAcceleration.times(TurretConstants.azimuthHoodCoupling));
        
        return new VariableMatrix(new Variable[][] {
            new Variable[] {
                // d(flywheel velocity)/dt
                flywheelAcceleration,
                // d(hood position)/dt = hood velocity
                state.get(2, 0),
                // d(hood velocity)/dt
                hoodAcceleration,
                // d(azimuth position)/dt = azimuth velocity
                state.get(4, 0),
                // d(azimuth velocity)/dt
                azimuthAcceleration
            }
        });
    }

    private Matrix<N5, N1> mcpDynamicsButNumbers(double[] state, double[] input) {
        double flywheelSteadyStateCurrent = TurretTuningData.FlywheelCurrentModel.calculate(input[0], input[1], input[2]);
        double hoodSteadyStateCurrent = TurretTuningData.HoodCurrentModel.calculate(input[0], input[1], input[2]);
        double azimuthSteadyStateCurrent = TurretTuningData.AzimuthCurrentModel.calculate(input[0], input[1], input[2]);

        double flywheelCurrent = input[0] - flywheelSteadyStateCurrent;
        double hoodCurrent = input[1] - hoodSteadyStateCurrent;
        double azimuthCurrent = input[2] - azimuthSteadyStateCurrent;

        double flywheelMotorAcceleration = flywheelCurrent * TurretConstants.flywheelSimMotor.KtNMPerAmp / TurretConstants.flywheelMotorInertiaKgM2;
        double hoodMotorAcceleration = hoodCurrent * TurretConstants.hoodSimMotor.KtNMPerAmp / TurretConstants.hoodMotorInertiaKgM2;
        double azimuthMotorAcceleration = azimuthCurrent * TurretConstants.azimuthSimMotor.KtNMPerAmp / TurretConstants.azimuthMotorInertiaKgM2;

        double azimuthAcceleration = azimuthMotorAcceleration * TurretConstants.totalAzimuthGearing * TurretConstants.totalAzimuthGearing;

        double flywheelAcceleration = flywheelMotorAcceleration * TurretConstants.totalFlywheelGearing * TurretConstants.totalFlywheelGearing
            + azimuthAcceleration * TurretConstants.azimuthFlyCoupling;
        double hoodAcceleration = hoodMotorAcceleration * TurretConstants.totalHoodGearing * TurretConstants.totalHoodGearing
            + azimuthAcceleration * TurretConstants.azimuthHoodCoupling;

        return VecBuilder.fill(flywheelAcceleration, state[2], hoodAcceleration, state[4], azimuthAcceleration);
    }

    private Variable mpcCost(VariableMatrix state, VariableMatrix input, double[] reference) {
        // Cost is basically just tracking error, but we add small penalties for current usage
        // Normalize to avoid overemphasizing flywheel
        Variable flywheelVelocityError = state.get(0, 0).minus(reference[0])
            .times(1 / TurretConstants.maxFlywheelSpeedRadPerSec * 0.5);
        Variable hoodPositionError = state.get(1, 0).minus(reference[1]);
        Variable azimuthPositionError = state.get(3, 0).minus(reference[2]);
        Variable cost = flywheelVelocityError.times(flywheelVelocityError)
            .plus(hoodPositionError.times(hoodPositionError))
            .plus(azimuthPositionError.times(azimuthPositionError));
        // for(int i = 0; i < 3; i++) {
        //     Variable current = input.get(i, 0);
        //     cost = cost.plus(current.times(current).times(0.001));
        // }
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
        problem.subjectTo(Constraints.lt(flywheelCurrent, TurretConstants.flywheelCurrentLimit));
        problem.subjectTo(Constraints.lt(hoodCurrent, TurretConstants.hoodCurrentLimit));
        problem.subjectTo(Constraints.lt(azimuthCurrent, TurretConstants.azimuthCurrentLimit));
    }

    private InitialGuess mpcInitialGuess(double[] currentState, double[] reference, int samplesN) {
        return new InitialGuess(getCurrentState().getData(), new double[] {
            0, 0, 0
        });
    }
}
