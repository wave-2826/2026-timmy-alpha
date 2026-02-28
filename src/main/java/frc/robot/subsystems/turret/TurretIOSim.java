package frc.robot.subsystems.turret;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.turret.TurretSim.TurretSimMode;

public class TurretIOSim extends TurretIOReal {
    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    protected static DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    // Spark simulation objects
    protected SparkSim flywheelMotorSim = new SparkFlexSim(topFlywheelMotor, flywheelSimMotor);
    protected SparkSim azimuthMotorSim = new SparkFlexSim(azimuthMotor, azimuthSimMotor);
    protected SparkSim hoodMotorSim = new SparkFlexSim(hoodMotor, hoodSimMotor);

    // Spark simulation sensors
    protected SparkAbsoluteEncoderSim azimuthEncoderSim = azimuthMotorSim.getAbsoluteEncoderSim();

    protected TurretSim turretSim = new TurretSim(TurretSimMode.MeasuredDynamics);

    public TurretIOSim() {
        super();
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        // Inputs should already bet set, but the top/bottom flywheels need to balance the top sim's
        // since we model them as one motor.
        var distributedFlywheel = inputs.topFlywheel.half();
        inputs.topFlywheel = distributedFlywheel;
        inputs.bottomFlywheel = distributedFlywheel;

        var turretState = turretSim.updateAndGetState(
            flywheelSimMotor.getTorque(flywheelMotorSim.getMotorCurrent()),
            hoodSimMotor.getTorque(hoodMotorSim.getMotorCurrent()),
            azimuthSimMotor.getTorque(azimuthMotorSim.getMotorCurrent()),
            0.02
        );
        
        // TODO: update sim thingies

        super.updateInputs(inputs);
    }
}
