package frc.robot.subsystems.turret.controller;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import frc.robot.subsystems.turret.Turret.TurretTarget;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

/**
 * Turret controller that runs the heavy calculation on a separate thread.
 * getOutput will block up to 10ms waiting for the latest computation and will
 * warn and return the last-known output on timeout.
 */
public class TurretControllerThreaded implements TurretControllerIO {
	private TurretController controller;
	private volatile ControlResult latestOutput = new ControlResult(new TurretMPCOutputs(0.0, 0.0, 0.0), 0.0);

    private static record ControlResult(
        TurretMPCOutputs outputs,
        double computationTimeMs
    ) {}

	private final ExecutorService executor = Executors.newSingleThreadExecutor(r -> {
		Thread t = new Thread(r, "TurretControllerThread");
		t.setDaemon(true);
		return t;
	});

	// The most recent future representing an in-flight computation. Starts completed
	// so getOutput can return immediately before any run() call.
	private volatile CompletableFuture<ControlResult> latestFuture = CompletableFuture.completedFuture(latestOutput);

	@Override
	public void init(TurretIOInputs inputs) {
		controller = new TurretController(inputs);
	}

	@Override
	public void getOutput(TurretControllerIOInputs inputs) {
		var f = latestFuture;
		try {
			var out = f.get(10, TimeUnit.MILLISECONDS);
			inputs.mpc = out.outputs;
            inputs.computationTimeMs = out.computationTimeMs;
		} catch(TimeoutException e) {
			// Warn and return the last known output on timeout
			System.err.println("[TurretControllerThreaded] Warning: timeout waiting for controller outputs (>10ms); using last known output");
			inputs.mpc = latestOutput.outputs;
            inputs.computationTimeMs = latestOutput.computationTimeMs;
		} catch(InterruptedException e) {
			Thread.currentThread().interrupt();
			inputs.mpc = latestOutput.outputs;
            inputs.computationTimeMs = latestOutput.computationTimeMs;
		} catch(ExecutionException e) {
			System.err.println("[TurretControllerThreaded] Error computing outputs: " + e.getCause());
			inputs.mpc = latestOutput.outputs;
            inputs.computationTimeMs = latestOutput.computationTimeMs;
		}
	}

	@Override
	public void run(TurretTarget target) {
		if(controller == null) {
			throw new IllegalStateException("TurretControllerThreaded not initialized");
		}

		final CompletableFuture<ControlResult> future = new CompletableFuture<>();
		latestFuture = future;

		// Submit the calculation to the background thread
		executor.submit(() -> {
			try {
                long startTime = System.nanoTime();
				
                double[] outputs = controller.getOutputs(target.azimuthAngleRad, target.hoodAngleRad, target.flywheelSpeedRadPerSec);
				
                TurretMPCOutputs out = new TurretMPCOutputs(outputs[0], outputs[1], outputs[2]);
                
                double computationTime = (double)(System.nanoTime() - startTime) / 1e6;
				latestOutput = new ControlResult(out, computationTime);
                
				future.complete(latestOutput);
			} catch (Throwable t) {
				future.completeExceptionally(t);
			}
		});
	}

}
