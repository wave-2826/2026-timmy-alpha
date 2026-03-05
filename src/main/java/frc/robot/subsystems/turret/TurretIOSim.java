package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.turret.controller.TurretControllerIO.TurretMPCOutputs;
import frc.robot.subsystems.turret.sim.TurretSim;
import frc.robot.subsystems.turret.sim.TurretSimDCMotor;

public class TurretIOSim extends TurretIOReal {
    private static int subticks = 10;

    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    protected static DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    // Current control PID loops - for some reason, the spark ControlMode.kCurrent doesn't work in sim??
    protected PIDController flywheelCurrentController = new PIDController(0, 0, 0, 0.02 / subticks);
    protected PIDController azimuthCurrentController = new PIDController(0, 0, 0, 0.02 / subticks);
    protected PIDController hoodCurrentController = new PIDController(0, 0, 0, 0.02 / subticks);
    protected Double flywheelCurrentTarget = null;
    protected Double azimuthCurrentTarget = null;
    protected Double hoodCurrentTarget = null;

    // Spark simulation objects
    protected SparkSim flywheelMotorSim = new SparkFlexSim(topFlywheelMotor, flywheelSimMotor);
    protected SparkSim azimuthMotorSim = new SparkFlexSim(azimuthMotor, azimuthSimMotor);
    protected SparkSim hoodMotorSim = new SparkFlexSim(hoodMotor, hoodSimMotor);

    // Spark simulation sensors
    protected SparkAbsoluteEncoderSim azimuthEncoderSim = azimuthMotorSim.getAbsoluteEncoderSim();

    protected TurretSim turretSim = new TurretSimDCMotor();

    public TurretIOSim() {
        super();

        TurretConstants.flywheelMotorPID.configureController(flywheelCurrentController, ClosedLoopSlot.kSlot1);
        TurretConstants.azimuthMotorPID.configureController(azimuthCurrentController, ClosedLoopSlot.kSlot1);
        TurretConstants.hoodMotorPID.configureController(hoodCurrentController, ClosedLoopSlot.kSlot1);
    }

    private double calculateTorque(SparkSim motorSim, DCMotor simMotor) {
        // double voltage = motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
        // // V * Kt / R gives us torque disregarding bEMF, which is already accounted for in the simulation's velocity update
        // return voltage * simMotor.KtNMPerAmp / simMotor.rOhms
        return simMotor.getTorque(motorSim.getMotorCurrent());
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        if(!DriverStationSim.getDsAttached()) {
            turretSim.reset();
            return;
        }

        // TODO: subtick on main subtick loop

        for(int i = 0; i < subticks; i++) {
            var turretState = turretSim.updateAndGetState(
                flywheelMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage(),
                hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage(),
                azimuthMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage(),
                0.02 / subticks
            );

            if(flywheelCurrentTarget != null) {
                double flyOutput = flywheelCurrentController.calculate(flywheelMotorSim.getMotorCurrent(), flywheelCurrentTarget);
                flywheelController.setSetpoint(flyOutput, ControlType.kDutyCycle);
            }
            if(azimuthCurrentTarget != null) {
                double azimuthOutput = azimuthCurrentController.calculate(azimuthMotorSim.getMotorCurrent(), azimuthCurrentTarget);
                azimuthController.setSetpoint(azimuthOutput, ControlType.kDutyCycle);
            }
            if(hoodCurrentTarget != null) {
                double hoodOutput = hoodCurrentController.calculate(hoodMotorSim.getMotorCurrent(), hoodCurrentTarget);
                hoodController.setSetpoint(hoodOutput, ControlType.kDutyCycle);
            }

            flywheelMotorSim.iterate(turretState.flywheelMotorVelRps(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
            hoodMotorSim.iterate(turretState.hoodMotorVelRps(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
            azimuthMotorSim.iterate(turretState.azimuthMotorVelRps(), RoboRioSim.getVInVoltage(), 0.02 / subticks);
        }

        var state = turretSim.getState();
        Logger.recordOutput("TurretSim/State", state);
        Logger.recordOutput("TurretSim/Motors/FlywheelVelRPS", state.flywheelMotorVelRps(), RotationsPerSecond);
        Logger.recordOutput("TurretSim/Motors/HoodVelRPS", state.hoodMotorVelRps(), RotationsPerSecond);
        Logger.recordOutput("TurretSim/Motors/AzimuthVelRPS", state.azimuthMotorVelRps(), RotationsPerSecond);

        Logger.recordOutput("TurretSim/FlyTorque", calculateTorque(flywheelMotorSim, flywheelSimMotor));
        Logger.recordOutput("TurretSim/HoodTorque", calculateTorque(hoodMotorSim, hoodSimMotor));
        Logger.recordOutput("TurretSim/AzimuthTorque", calculateTorque(azimuthMotorSim, azimuthSimMotor));

        Logger.recordOutput("TurretSim/Setpoints/FlySetpoint", flywheelMotorSim.getSetpoint());
        Logger.recordOutput("TurretSim/Setpoints/HoodSetpoint", hoodMotorSim.getSetpoint());
        Logger.recordOutput("TurretSim/Setpoints/AzimuthSetpoint", azimuthMotorSim.getSetpoint());

        azimuthEncoderSim.setVelocity(turretSim.getState().azimuthVelRps() * 2 * Math.PI);
        azimuthEncoderSim.setPosition(turretSim.getState().azimuthPosRad() * 2 * Math.PI);

        super.updateInputs(inputs);

        // Inputs should already bet set, but the top/bottom flywheels need to balance the top sim's
        // since we model them as one motor.
        var distributedFlywheel = inputs.topFlywheel.half();
        inputs.topFlywheel = distributedFlywheel;
        inputs.bottomFlywheel = distributedFlywheel;
    }

    // private double manualVoltageCompensation(double voltage) {
    //     return voltage * 13.4 / RobotController.getBatteryVoltage();
    // }
    // private double getVoltage(DCMotor motor, double current, double speedRadiansPerSec) {
    //     return manualVoltageCompensation(1.0 / motor.KvRadPerSecPerVolt * speedRadiansPerSec) +
    //         motor.rOhms * current;
    // }

    @Override
    public void setPIDOutputs(TurretIOPIDOutputs outputs) {
        flywheelCurrentTarget = null;
        hoodCurrentTarget = null;
        azimuthCurrentTarget = null;
        super.setPIDOutputs(outputs);
    }

    @Override
    public void setMPCOutputs(TurretMPCOutputs outputs) {
        flywheelCurrentTarget = outputs.flywheelCurrent();
        hoodCurrentTarget = outputs.hoodCurrent();
        azimuthCurrentTarget = outputs.azimuthCurrent();
    }
}
