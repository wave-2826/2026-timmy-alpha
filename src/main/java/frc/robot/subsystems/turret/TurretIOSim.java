package frc.robot.subsystems.turret;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TurretIOSim extends TurretIOReal {
    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    protected static DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    // Spark simulation objects
    protected SparkSim topFlywheelMotorSim = new SparkFlexSim(topFlywheelMotor, flywheelSimMotor);
    protected SparkSim bottomFlywheelMotorSim = new SparkFlexSim(bottomFlywheelMotor, flywheelSimMotor);
    protected SparkSim azimuthMotorSim = new SparkFlexSim(azimuthMotor, azimuthSimMotor);
    protected SparkSim hoodMotorSim = new SparkFlexSim(hoodMotor, hoodSimMotor);

    // Spark simulation sensors
    protected SparkAbsoluteEncoderSim azimuthEncoderSim = azimuthMotorSim.getAbsoluteEncoderSim();

    protected TurretSim turretSim = new TurretSim();

    public TurretIOSim() {
        super();
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        // Outputs should already be set
        

        super.updateInputs(inputs);
    }
}
