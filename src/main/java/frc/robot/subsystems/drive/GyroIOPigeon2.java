package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.pigeonId, DriveConstants.CANBus);
    
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    private double previousYawVelocity = 0.0;
    /** Previous update timestamp in seconds */
    private double previousTimestamp;

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);

        yaw.setUpdateFrequency(DriveConstants.odometryFrequency);
        yawVelocity.setUpdateFrequency(50.0);
        previousTimestamp = yaw.getTimestamp().getTime();
        
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());

        // There's no signal for angular acceleration?? we have to calculate it manually...
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        double yawTs = yawVelocity.getTimestamp().getTime();
        inputs.yawAccelerationRadPerSec2 = (inputs.yawVelocityRadPerSec - previousYawVelocity) / (yawTs - previousTimestamp);
        previousTimestamp = yawTs;

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
        
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
