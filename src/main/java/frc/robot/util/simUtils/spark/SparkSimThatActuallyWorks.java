package frc.robot.util.simUtils.spark;

import com.revrobotics.sim.MovingAverageFilterSim;
import com.revrobotics.sim.NoiseGenerator;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/**
 * Why is rev's spark sim so terrible at control loops?? Maybe I'm being dumb but I genuinely
 * can't figure out how to use the API in a way that produses outputs that match real life.
 */
public class SparkSimThatActuallyWorks {
    private final SparkBase spark;
    private final String deviceName;
    private final DCMotor simMotor;

    private final SimDouble appliedOutput;
    private final SimDouble velocity;
    private final SimDouble position;
    private final SimDouble busVoltage;
    private final SimDouble motorCurrent;
    private final SimDouble setpoint;
    
    private final SimDouble arbFF;
    private final SimEnum arbFFUnits;
    private final ArbFFUnits getArbFFUnits() {
        return ArbFFUnits.values()[arbFFUnits.get()];
    }

    private final SimInt closedLoopSlot;
    private final ClosedLoopSlot getClosedLoopSlot() {
        return ClosedLoopSlot.values()[closedLoopSlot.get()];
    }
    private final SimEnum controlMode;
    private final ControlType getControlType() {
        return ControlType.values()[controlMode.get()];
    }

    private final MovingAverageFilterSim velocityAverage = new MovingAverageFilterSim(2, 0.016);

    private SparkBaseConfigAccessor configAcc;

    private PIDController controller = null;
    private double controllerIntegratorRange = 0;
    private void updateControllerIfNeeded(double dt) {
        ClosedLoopSlot slot = getClosedLoopSlot();
        double kP = configAcc.closedLoop.getP(slot),
            kI = configAcc.closedLoop.getI(slot),
            kD = configAcc.closedLoop.getD(slot),
            kIMaxAcc = configAcc.closedLoop.getMaxIAccumulation(slot),
            kIZone = configAcc.closedLoop.getIZone(slot),
            posWrapMin = configAcc.closedLoop.getPositionWrappingMinInput(),
            posWrapMax = configAcc.closedLoop.getPositionWrappingMaxInput();
        boolean posWrapEnabled = configAcc.closedLoop.getPositionWrappingEnabled();
            
        if(controller == null || controller.getPeriod() != dt) {
            controller = new PIDController(kP, kI, kD, dt);
            controller.setIntegratorRange(-kIMaxAcc, kIMaxAcc);
            controllerIntegratorRange = kIMaxAcc;
        }
        if(controller.getP() != kP || controller.getI() != kI || controller.getD() != kD) {
            controller.setPID(kP, kI, kD);
            controller.reset();
        }
        if(controller.getIZone() != kIZone) {
            controller.setIZone(kIZone);
            controller.reset();
        }
        if(controllerIntegratorRange != kIMaxAcc) {
            controller.setIntegratorRange(-kIMaxAcc, kIMaxAcc);
            controllerIntegratorRange = kIMaxAcc;
            controller.reset();
        }

        if(posWrapEnabled && getControlType() == ControlType.kPosition) {
            controller.enableContinuousInput(posWrapMin, posWrapMax);
        } else {
            controller.disableContinuousInput();
        }
    }

    private int setupSimDeviceJNI() {
        int id = SimDeviceJNI.createSimDevice(deviceName);
        SimDeviceJNI.createSimValueDouble(id, "Applied Output", SimDeviceJNI.kOutput, 0);
        SimDeviceJNI.createSimValueDouble(id, "Position", SimDeviceJNI.kOutput, 0);
        SimDeviceJNI.createSimValueDouble(id, "Velocity", SimDeviceJNI.kOutput, 0);
        SimDeviceJNI.createSimValueDouble(id, "Bus Voltage", SimDeviceJNI.kInput, 12.0);
        SimDeviceJNI.createSimValueDouble(id, "Motor Current", SimDeviceJNI.kOutput, 0);
        SimDeviceJNI.createSimValueDouble(id, "Setpoint", SimDeviceJNI.kOutput, 0);
        
        SimDeviceJNI.createSimValueDouble(id, "Arbitrary Feedforward", SimDeviceJNI.kOutput, 0);
        SimDeviceJNI.createSimValueEnum(id, "ArbFF Units", SimDeviceJNI.kBidir, new String[]{
            "kVoltage",
            "kPercentOut"
        }, 0);

        SimDeviceJNI.createSimValueInt(id, "Closed Loop Slot", SimDeviceJNI.kBidir, 0);
        SimDeviceJNI.createSimValueEnum(id, "Control Mode", SimDeviceJNI.kBidir, new String[]{
            "kDutyCycle",
            "kVelocity",
            "kVoltage",
            "kPosition",
            "kCurrent",
            "kMAXMotionPositionControl",
            "kMAXMotionVelocityControl"
        }, 0);

        return id;
    }

    // Never ever call .iterate() on this. That's what got us into this mess.
    private final SparkSim internalSim;
    
    public SparkSimThatActuallyWorks(SparkBase spark, String customName, DCMotor simMotor) {
        this.spark = spark;
        this.deviceName = "Custom Spark Sim " + customName + " (" + spark.getDeviceId() + ")";
        this.simMotor = simMotor;

        if(spark instanceof SparkFlex) {
            configAcc = ((SparkFlex)spark).configAccessor;
        } else {
            configAcc = ((SparkMax)spark).configAccessor;
        }

        int id = setupSimDeviceJNI();
        SimDeviceSim sparkSim = new SimDeviceSim(id);

        appliedOutput = sparkSim.getDouble("Applied Output");
        position = sparkSim.getDouble("Position");
        velocity = sparkSim.getDouble("Velocity");
        busVoltage = sparkSim.getDouble("Bus Voltage");
        motorCurrent = sparkSim.getDouble("Motor Current");
        setpoint = sparkSim.getDouble("Setpoint");
        
        arbFF = sparkSim.getDouble("Arbitrary Feedforward");
        arbFFUnits = sparkSim.getEnum("ArbFF Units");
        
        closedLoopSlot = sparkSim.getInt("Closed Loop Slot");
        controlMode = sparkSim.getEnum("Control Mode");

        internalSim = new SparkSim(spark, simMotor);
    }

    private boolean runLimitLogic(boolean forward) {
        return false; // whatever for now
    }

    // hack hack hack hack ugh
    private void copyFromSparkSimEnum(SimEnum val, String paramName) {
        try {
            var field = SparkSim.class.getDeclaredField(paramName);
            field.setAccessible(true);

            SimInt sparkSimVal = (SimInt)field.get(internalSim);
            val.set(sparkSimVal.get());
        } catch(NoSuchFieldException | IllegalAccessException e) {
            DriverStation.reportError("Error copying value from SparkSim: " + e.getMessage(), false);
        }
    }
    private void copyFromSparkSimInt(SimInt val, String paramName) {
        try {
            var field = SparkSim.class.getDeclaredField(paramName);
            field.setAccessible(true);

            SimInt sparkSimVal = (SimInt)field.get(internalSim);
            val.set(sparkSimVal.get());
        } catch(NoSuchFieldException | IllegalAccessException e) {
            DriverStation.reportError("Error copying value from SparkSim: " + e.getMessage(), false);
        }
    }
    private void copyFromSparkSimDouble(SimDouble val, String paramName) {
        try {
            var field = SparkSim.class.getDeclaredField(paramName);
            field.setAccessible(true);

            SimDouble sparkSimVal = (SimDouble)field.get(internalSim);
            val.set(sparkSimVal.get());
        } catch(NoSuchFieldException | IllegalAccessException e) {
            DriverStation.reportError("Error copying value from SparkSim: " + e.getMessage(), false);
        }
    }

    private double getPositionFactor() {
        double positionFactor;
        
        // We just assume spark flex. Whatever. It works either way.
        if(configAcc.closedLoop.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder) {
            positionFactor = configAcc.absoluteEncoder.getPositionConversionFactor();
        } else {
            positionFactor = configAcc.encoder.getPositionConversionFactor();
        }

        if(positionFactor == 0.0) positionFactor = 1.0;
        return positionFactor;
    }
    private double getVelocityFactor() {
        double velocityFactor;
        
        // We just assume spark flex. Whatever. It works either way.
        if(configAcc.closedLoop.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder) {
            velocityFactor = configAcc.absoluteEncoder.getVelocityConversionFactor();
        } else {
            velocityFactor = configAcc.encoder.getVelocityConversionFactor();
        }

        if(velocityFactor == 0.0) velocityFactor = 1.0;
        return velocityFactor;
    }

    public void iterate(double simVelocity, double vbus, double dt) {
        double positionFactor = getPositionFactor();
        double velocityFactor = getVelocityFactor();

        // Bare minimum we can't do ourselves
        // We're really really not supposed to do this, but... I gave up on any other strategy
        copyFromSparkSimInt(closedLoopSlot, "m_closedLoopSlot");
        copyFromSparkSimDouble(setpoint, "m_setpoint");
        copyFromSparkSimEnum(controlMode, "m_controlMode");
        copyFromSparkSimDouble(arbFF, "m_arbFF");
        copyFromSparkSimEnum(arbFFUnits, "m_arbFFUnits");

        updateControllerIfNeeded(dt);

        // Velocity input is the system simulated input.
        double internalVelocity = NoiseGenerator.hallSensorVelocity(simVelocity);

        // technically velocityAverage should be updated with the uvw constants, but whatever
        velocityAverage.put(internalVelocity, dt);
        internalVelocity = velocityAverage.get();

        // First set the states that are given
        velocity.set(internalVelocity);

        double velocityRPM = simVelocity / velocityFactor;

        position.set(position.get() + ((velocityRPM / 60) * dt) * positionFactor);
        busVoltage.set(vbus);

        // Calculate the applied output
        double appliedOutput = 0.0;
        switch(getControlType()) {
            case kDutyCycle:
                appliedOutput = setpoint.get();
                break;
            case kVelocity:
                double feedbackVelocity;
                if(configAcc.closedLoop.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder) {
                    feedbackVelocity = spark.getAbsoluteEncoder().getVelocity();
                } else {
                    feedbackVelocity = velocity.get();
                }
                appliedOutput = controller.calculate(feedbackVelocity, setpoint.get());
                break;
            case kCurrent:
                appliedOutput = controller.calculate(motorCurrent.get(), setpoint.get());
                break;
            case kPosition:
                double feedbackPosition;
                if(configAcc.closedLoop.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder) {
                    feedbackPosition = spark.getAbsoluteEncoder().getPosition();
                } else {
                    feedbackPosition = position.get();
                }
                appliedOutput = controller.calculate(feedbackPosition, setpoint.get());
                break;
            case kVoltage:
                appliedOutput = setpoint.get() / vbus;
                break;
            
            case kMAXMotionPositionControl:
            case kMAXMotionVelocityControl:
            default:
                // Unsupported
                DriverStation.reportError("Unsupported control mode " + getControlType(), false);
                appliedOutput = 0;
                break;
        }

        switch(getArbFFUnits()) {
            case kPercentOut:
                appliedOutput += arbFF.get();
                break;
            case kVoltage:
                appliedOutput += arbFF.get() / vbus;
                break;
        }

        // I have NO IDEA what closed loop mode does, and it seems neither does Rev.
        // We just use "on"/"off" like the configuration allows.
        if(configAcc.getVoltageCompensationEnabled()) {
            double voltageComp = configAcc.getVoltageCompensation();
            appliedOutput = (appliedOutput * voltageComp) / vbus;
        }

        // Limit to [-1, 1] or limit switch value
        double maxOutput = runLimitLogic(true) ? 0 : 1;
        double minOutput = runLimitLogic(false) ? 0 : -1;
        appliedOutput = Math.min(Math.max(appliedOutput, minOutput), maxOutput);

        // See 12.1.3; Rev's current limiting is a black box, but this is probably close enough.
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf
        // This doesn't account for:
        // - Limits other than smart current limit
        // - Dynamic RPM/free smart current limit
        
        double requestedOutputVoltageVolts = appliedOutput * vbus;
        double limitedVoltage = requestedOutputVoltageVolts;
        double currentLimitAmps = configAcc.getSmartCurrentLimit();
        final double kCurrentThreshold = 1.1;
        if(Math.abs(motorCurrent.get()) > kCurrentThreshold * currentLimitAmps) {
            limitedVoltage = simMotor.getVoltage(
                simMotor.getTorque(Math.copySign(currentLimitAmps, motorCurrent.get())),
                Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM)
            );
        }

        // ensure the current limit doesn't cause an increase to output voltage
        if(Math.abs(limitedVoltage) > Math.abs(requestedOutputVoltageVolts))
            limitedVoltage = requestedOutputVoltageVolts;
        
        appliedOutput = limitedVoltage / vbus;

        // naive motor model current calculation
        motorCurrent.set(simMotor.getCurrent(
            Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM), appliedOutput * vbus
        ));

        // check for faults
        SparkBase.Faults motorFaults = spark.getFaults();
        SparkBase.Faults motorStickyFaults = spark.getStickyFaults();
        if(motorFaults.can || motorStickyFaults.can || motorFaults.escEeprom || motorStickyFaults.escEeprom || motorFaults.motorType || motorStickyFaults.motorType || motorFaults.firmware || motorStickyFaults.firmware || motorFaults.gateDriver || motorStickyFaults.gateDriver || motorFaults.sensor || motorStickyFaults.sensor || motorFaults.temperature || motorStickyFaults.temperature || motorFaults.other || motorStickyFaults.other) {
            appliedOutput = 0;
            DriverStation.reportWarning(deviceName + ": Sim spark stopped due to fault", false);
        }

        // Enable logic - we don't simulate .enable() and .disable()
        if(DriverStation.isEnabled()) {
            this.appliedOutput.set(appliedOutput);
        } else {
            this.appliedOutput.set(0.0);
            motorCurrent.set(0.0);
        }

        // Update SparkSim
        internalSim.setAppliedOutput(this.appliedOutput.get());
        internalSim.setBusVoltage(this.busVoltage.get());
        internalSim.setMotorCurrent(this.motorCurrent.get());
        internalSim.setPosition(this.position.get());
        internalSim.setVelocity(this.velocity.get());

        internalSim.getRelativeEncoderSim().setPosition(this.position.get());
        internalSim.getRelativeEncoderSim().setVelocity(this.velocity.get());
    }

    public void setPosition(double position) {
        this.position.set(position);
    }

    public double getMotorCurrent() {
        return motorCurrent.get();
    }

    public double getAppliedOutput() {
        return appliedOutput.get();
    }

    public double getSetpoint() {
        return setpoint.get() * getPositionFactor();
    }

    public SparkAbsoluteEncoderSim getAbsoluteEncoderSim() {
        return internalSim.getAbsoluteEncoderSim();
    }
}
