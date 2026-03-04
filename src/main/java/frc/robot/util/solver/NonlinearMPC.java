package frc.robot.util.solver;

import java.util.function.BiFunction;
import org.ejml.simple.SimpleMatrix;
import org.wpilib.math.autodiff.NumericalIntegration;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.Options;

/**
 * A nonlinear model-predictive control solver using Slepnir based on that presented in [Controls Engineering in FRC](https://file.tavsys.net/control/controls-engineering-in-frc.pdf)
 * The book's Python implementation is available here: https://github.com/calcmogul/controls-engineering-in-frc/blob/main/bookutil/bookutil/nonlinear_mpc.py
 */
public class NonlinearMPC {
    // Helper interfaces and classes
    public static class InitialGuess {
        public final SimpleMatrix X;
        public final SimpleMatrix U;

        public InitialGuess(SimpleMatrix X, SimpleMatrix U) {
            this.X = X;
            this.U = U;
        }
        public InitialGuess(double[] x, double[] u) {
            this.X = new SimpleMatrix(x.length, 1, true, x);
            this.U = new SimpleMatrix(u.length, 1, true, u);
        }
    }

    public static interface InitialGuessFunction {
        InitialGuess apply(double[] currentState, double[] reference, int samplesN);
    }
    public static interface CostFunction {
        Variable apply(VariableMatrix currentState, VariableMatrix inputs, VariableMatrix reference);
    }
    public static interface ConstraintFunction {
        void apply(Problem problem, VariableMatrix currentState, VariableMatrix inputs);
    }
    public static interface DynamicsFunction extends BiFunction<VariableMatrix, VariableMatrix, VariableMatrix> {
        VariableMatrix apply(VariableMatrix state, VariableMatrix input);
    }

    private final int states;
    private final int inputs;
    private final DynamicsFunction f;
    private final double samplePeriod;
    private final CostFunction cost;
    private final ConstraintFunction constraints;
    private final InitialGuessFunction initialGuess;
    private final double tolerance;
    private final double timeout;
    private final int N;

    private boolean warmStartable = false;
    private SimpleMatrix XWarmStart;
    private SimpleMatrix UWarmStart;

    /**
     * Creates a new NonlinearMPC instance.
     * @param states Number of states in the system
     * @param inputs Number of inputs to the system
     * @param f Dynamics dx/dt = f(x, u).
     * @param samplePeriod Sample period in seconds.
     * @param cost Callback for cost function.
     * @param constraints Callback for setting constraints (control input limits, etc.).
     * @param initialGuess Callback that takes the current state x, reference r, and number of
     * samples N and returns the initial guesses for X (states x (N + 1)) and U (inputs x N).
     * Subsequent iterations warm start from the previous iteration's solution.
     * @param predictionHorizon Prediction horizon in seconds.
     * @param timeout The maximum time in seconds the solver can spend before returning a solution.
     * @param tolerance The tolerance for the solver's convergence. Larger values may result in faster solve times but worse solutions.
     */
    public NonlinearMPC(
        int states,
        int inputs,
        DynamicsFunction f,
        double samplePeriod,
        CostFunction cost,
        ConstraintFunction constraints,
        InitialGuessFunction initialGuess,
        double predictionHorizon,
        double timeout,
        double tolerance
    ) {
        this.states = states;
        this.inputs = inputs;
        this.f = f;
        this.samplePeriod = samplePeriod;
        this.cost = cost;
        this.constraints = constraints;
        this.initialGuess = initialGuess;
        this.tolerance = tolerance;
        this.timeout = timeout;
        this.N = (int) (predictionHorizon / samplePeriod);
    }

    public static VariableMatrix columnMatrix(Variable[] items) {
        Variable[][] list = new Variable[items.length][1];
        for(int i = 0; i < items.length; i++) list[i][0] = items[i];
        return new VariableMatrix(list);
    }
    public static VariableMatrix columnMatrix(double[] items) {
        Variable[][] list = new Variable[items.length][1];
        for(int i = 0; i < items.length; i++) list[i][0] = new Variable(items[i]);
        return new VariableMatrix(list);
    }

    private Problem problem;
    private VariableMatrix X;
    private VariableMatrix U;
    private VariableMatrix InitialState;
    private VariableMatrix CostReference;

    public double[] calculate(double[] x, double[] r) {
        // Construct the problem once and only if something changes that we can't 
        if(problem == null) {
            problem = new Problem();
            X = problem.decisionVariable(states, N + 1);
            U = problem.decisionVariable(inputs, N);
            InitialState = new VariableMatrix(states, 1);
            CostReference = new VariableMatrix(1, r.length);

            problem.minimize(cost.apply(X, U, CostReference));

            // Initial state constraint (will update value each call)
            problem.subjectTo(Constraints.eq(X.col(0), InitialState));

            // Dynamics constraints
            for(int k = 0; k < N; k++) {
                VariableMatrix nextX = NumericalIntegration.rk4(f, X.col(k), U.col(k), samplePeriod);
                problem.subjectTo(Constraints.eq(X.col(k), nextX));
            }

            constraints.apply(problem, X, U);
        }

        CostReference.set(new double[][] { r });
        for(int i = 0; i < x.length; i++) InitialState.set(i, x[i]);

        // Initial guess
        if(!warmStartable) {
            InitialGuess guess = initialGuess.apply(x, r, N);
            XWarmStart = guess.X;
            UWarmStart = guess.U;
            warmStartable = true;
        }

        X.set(XWarmStart);
        U.set(UWarmStart);

        problem.solve(new Options().withTimeout(timeout).withTolerance(tolerance));

        XWarmStart = X.value();
        UWarmStart = U.value();

        return UWarmStart.getZDRM().data;
    }
}
