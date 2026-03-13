# Control Techniques

In 2026, our team learned a whole lot about control techniques in persuit of controlling our triple-coaxial turret most efficiently. We tried a lot of different approaches, so here's a quick summary of our findings.

## PID Controllers

A PID (Proportional-Integral-Derivative) controller works by continuously calculating an error value as the difference between a desired setpoint and the measured process variable. It then applies a correction based on three terms:

- **Proportional (P):** This term produces an output proportional to the current error. The proportional gain ($K_p$) determines how strongly the controller reacts to the error. Higher $K_p$ means faster response, but too high can cause instability.
- **Integral (I):** This term sums the error over time, addressing accumulated offset. The integral gain ($K_i$) determines how aggressively the controller eliminates steady-state error. Too much $K_i$ can lead to overshoot and oscillation.
- **Derivative (D):** This term predicts future error based on its rate of change. The derivative gain ($K_d$) dampens the response, reducing overshoot and improving stability.

The controller output is calculated as:
$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

One must then tuning the gains $K_p$, $K_i$, and $K_d$; each affects the system's speed, stability, and accuracy.

PID controllers are a classic approach for single-input systems. They're simple to implement and tune, making them ideal for straightforward control tasks. However, when dealing with multiple-input, multiple-output (MIMO) systems, PID controllers can struggle to coordinate the interactions between variables, leading to suboptimal performance. For our turret, PID controllers often caused instability and the complicated friction interactions between stages made them incredibly difficult to tune. Furthermore, the controller gains aren't very closely based on physical properties, so they must be entirely re-tuned when the mechanism changes.

## Linear System Simulation

Simulating linear systems is a (righfully) common tool in control engineering. Linear systems feel pretty unapproachable as they're often represented with complicated matrices and equations, but they're fundamentally very simple. In a linear system, one represents the system with three vectors and four matrices:
- x: the state vector, which contains all the variables that describe the system's current state (e.g., position, velocity, etc.)
- u: the input vector, which contains all the control inputs to the system (e.g., motor voltages)
- y: the output vector, which contains all the variables that we can measure from the system (e.g., encoder readings)
    - often, y is just the state vector or a subset of it!

- A: the state transition matrix, which describes how the state evolves over time without any input. Basically, this matrix finds how the state changes based on itself.
- B: the input matrix, which describes how the control inputs affect the state. This matrix finds how the state changes based on the control inputs (e.g. acceleration from motor voltage)
- C: the output matrix, which describes how the state relates to the outputs. This matrix can simply be an identity to output the state variables.
- D: the feedforward matrix, which describes how the control inputs directly affect the outputs. Honestly, we aren't quite sure what systems this could be useful in. It's almost always just a zero matrix.

If you're familiar enough with matrix math to understand the rough interactions here, the system can then be modeled over time with two simple equations:
$$\begin{aligned}
x_{k+1} &= Ax_k + Bu_k \\
y_k &= Cx_k + Du_k
\end{aligned}$$

Basically, all this shows is [next state] = [how state changes from itself] * [current state] + [how state changes from input] * [control input], and [output] = [how output reflects current state] * [current state] + [how output changes directly with input] * [control input].

These linear models are powerful because linear systems are easy and fast to work with while still being expressive enough to model most first-order effects on mechanisms. With linear models, we can predict systems' responses to various inputs, calculate controllers informed by how systems truly work, and analyze stability. Even when the real-world system is nonlinear, linear approximations can be "good enough", as in the case of an arm affected by gravity.

## Model Predictive Control (MPC)

Model Predictive Control is an advanced technique that uses a model of the system to predict future behavior and optimize control actions over a fixed time horizon. MPC controllers are great at handling constraints and planning ahead, which can be valuable for systems like our turret (e.g. to proactively respond to balls being fed). We experimented with MPC and found it powerful, especially when integrated the [slepnir constraint solver](https://www.chiefdelphi.com/t/sleipnir-java-bindings/511533). Unfortunately, its computational demands were too high for the RoboRIO, making it impractical to use. With more powerful controllers in the future, MPC could become a viable option.

## Linear Quadratic Regulator (LQR)

LQR controllers strike a balance between performance and complexity. They optimize control inputs to minimize a cost function, resulting in smooth and efficient operation. LQR is particularly effective for MIMO systems and often outperforms PID while still being inexpensive to calculate. Thanks to WPILib's support, LQR controllers are accessible and easy to integrate into robotics projects. We found LQR to be a robust solution for our turret.

LQR controllers use a linear model of the system and a cost function based on state errors and control effort, ultimately computing the ideal matrix to act as a proportional controller that minimizes this cost over an infinite horizon.

The cost function $J$ is defined as follows, but its precise mathematical definition doesn't matter much! Just know that it balances the desire to stay close to the desired state with the desire to use minimal control effort (e.g. motor power).
$$
J = \sum_{k=0}^{\infty} \left( x_k^T Q x_k + u_k^T R u_k \right)
$$

Here, $x_k$ is the state vector, $u_k$ is the control input, $Q$ is a matrix weighting state errors, and $R$ is a matrix weighting control effort. The controller computes the optimal input as:

$$
u_k = -K x_k
$$

where $K$ is the gain matrix calculated from the system's dynamics and the chosen $Q$ and $R$. Its derivation is very complicated, but the magic of an LQR controller is that all of the inputs leading to it are intuitive and based on the physical properties of the system, so tuning is straightforward.