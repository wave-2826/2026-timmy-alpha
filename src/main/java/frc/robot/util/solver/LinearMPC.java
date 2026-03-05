package frc.robot.util.solver;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.system.Discretization;

import org.ejml.simple.SimpleMatrix;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Constraints;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.Options;

public class LinearMPC<States extends Num, Inputs extends Num> {
    /**
     * A describes how the system state naturally evolves over time: pos += velocity.  
     * This is the "linearization" part of linear MPC - we assume the system evolves as
     * a linear function of the current state and inputs, and then solve for the optimal
     * inputs under that assumption.  
     * This should not be multiplied by delta - the MPC will discretize it. 
     */
    private Matrix<States, States> Ad;

    /**
     * B describes how the system state evolves in response to inputs.  
     * This is descretized over the controller timestep.
     */
    private Matrix<States, Inputs> Bd;

    /**
     * Q is the cost matrix for the state error.
     * Higher values mean the controller will prioritize minimizing that component of the state error more.
     */
    private Vector<States> Q;

    /**
     * R is the cost matrix for the inputs.
     * Higher values mean the controller will prioritize minimizing that component of the input more.
     */
    private Vector<Inputs> R;

    /**
     * The horizon for the solver.
     */
    private int N;

    // Constraints
    private Vector<Inputs> minUValues, maxUValues;
    private Vector<States> minXValues, maxXValues;

    private int states;
    private int inputs;

    /**
     * Creates a new LinearMPC instance.
     * @param A System matrix <States x States>
     *   A describes how the system state naturally evolves over time: pos += velocity.  
     *   This is the "linearization" part of linear MPC - we assume the system evolves as
     *   a linear function of the current state and inputs, and then solve for the optimal
     *   inputs under that assumption.  
     *   This should not be multiplied by delta - the MPC will discretize it.
     * @param B Input matrix <States x Inputs>
     *   B describes how the system state evolves in response to inputs.  
     *   This should not be multiplied by delta - the MPC will discretize it.
     * @param Q State error cost matrix <States> - higher values mean the controller will prioritize minimizing that component of the state error more.
     * @param R Input cost matrix <Inputs> - higher values mean the controller will prioritize minimizing that component of the input more.
     * @param period Controller period, seconds
     * @param horizon Solver horizon, seconds
     */
    public LinearMPC(
        Matrix<States, States> A,
        Matrix<States, Inputs> B,
        Vector<States> Q,
        Vector<Inputs> R,
        double period,
        double horizon
    ) {
        var disc = Discretization.discretizeAB(A, B, period);
        this.Ad = disc.getFirst();
        this.Bd = disc.getSecond();

        states = Ad.getNumRows();
        inputs = Bd.getNumCols();
        
        this.Q = Q;
        this.R = R;
        this.N = (int) (horizon / period);

        minU = new VariableMatrix(inputs, 1);
        maxU = new VariableMatrix(inputs, 1);
        minX = new VariableMatrix(states, 1);
        maxX = new VariableMatrix(states, 1);

        ReferenceMatrix = new VariableMatrix(states, 1);
        CurrentStateMatrix = new VariableMatrix(states, 1);

        constructProblem();
    }

    public void setInputBounds(Vector<Inputs> min, Vector<Inputs> max) {
        this.minUValues = min;
        this.maxUValues = max;
    }

    public void setStateBounds(Vector<States> min, Vector<States> max) {
        this.minXValues = min;
        this.maxXValues = max;
    }

    private boolean shouldWarmStart = true;
    private SimpleMatrix XWarmStart;
    private SimpleMatrix UWarmStart;

    private VariableMatrix AdMatrix, BdMatrix;

    private VariableMatrix minU, maxU, minX, maxX;
    private VariableMatrix ReferenceMatrix, CurrentStateMatrix;
    private VariableMatrix X, U;
    private Problem problem;

    private void constructProblem() {
        problem = new Problem();

        X = problem.decisionVariable(states, N + 1);
        U = problem.decisionVariable(inputs, N);

        // Cost
        Variable cost = new Variable(0.0);
        for(int k = 0; k < N; k++) {
            for(int i = 0; i < states; i++) {
                Variable err = X.get(i, k).minus(ReferenceMatrix.get(i));
                cost = cost.plus(err.times(err).times(Q.get(i)));
            }
            for(int i = 0; i < inputs; i++) {
                Variable uVal = U.get(i, k);
                cost = cost.plus(uVal.times(uVal).times(R.get(i)));
            }
        }
        problem.minimize(cost);

        // Initial state constraint
        for(int i = 0; i < states; i++) {
            problem.subjectTo(Constraints.eq(X.get(i, 0), CurrentStateMatrix.get(i)));
        }

        AdMatrix = new VariableMatrix(Ad.getStorage());
        BdMatrix = new VariableMatrix(Bd.getStorage());

        // Dynamics constraints
        for(int k = 0; k < N; k++) {
            problem.subjectTo(Constraints.eq(
                X.col(k + 1),
                AdMatrix.times(X.col(k)).plus(BdMatrix.times(U.col(k)))
            ));
        }

        // Apply Constraints (Bounds)
        for(int k = 0; k < N; k++) {
            for(int i = 0; i < inputs; i++) {
                problem.subjectTo(Constraints.ge(U.get(i, k), minU.get(i, 0)));
                problem.subjectTo(Constraints.le(U.get(i, k), maxU.get(i, 0)));
            }
            for(int i = 0; i < states; i++) {
                problem.subjectTo(Constraints.ge(X.get(i, k), minX.get(i, 0)));
                problem.subjectTo(Constraints.le(X.get(i, k), maxX.get(i, 0)));
            }
        }
    }

    public double[] calculate(Vector<States> currentState, Vector<States> reference) {
        if(shouldWarmStart) {
            XWarmStart = new SimpleMatrix(states, N + 1);
            UWarmStart = new SimpleMatrix(inputs, N);
            
            // Basic initial guess
            for(int i = 0; i < states; i++) {
                for(int j = 0; j < N + 1; j++) {
                    XWarmStart.set(i, j, currentState.get(i));
                }
            }

            shouldWarmStart = false;
        }

        CurrentStateMatrix.set(currentState.getStorage());
        ReferenceMatrix.set(reference.getStorage());

        minU.set(minUValues.getStorage());
        maxU.set(maxUValues.getStorage());
        minX.set(minXValues.getStorage());
        maxX.set(maxXValues.getStorage());

        X.set(XWarmStart);
        U.set(UWarmStart);

        constructProblem();

        Options options = new Options(); //.withTimeout(0.02).withTolerance(0.1);
        problem.solve(options);

        // System.out.println(U.value());

        XWarmStart = X.value();
        UWarmStart = U.value();

        return UWarmStart.getColumn(0).getDDRM().getData();
    }
}
