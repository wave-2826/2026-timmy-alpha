package frc.robot.util.tunables;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.OptionalDouble;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import static frc.robot.util.SparkUtil.tryUntilOk;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.GenericPIDConstants;
import frc.robot.util.GenericPIDConstants.PIDSlot;
import frc.robot.util.PhoenixUtil;

public class TunablePID {
    /** A set of PID constants with tunable numbers for each for logged tunable PIDs. */
    public class InternalPIDConstants {
        public LoggedTunableNumber p = null;
        public LoggedTunableNumber i = null;
        public LoggedTunableNumber d = null;

        public LoggedTunableNumber iZone = null;

        public LoggedTunableNumber fkS = null;
        public LoggedTunableNumber fkV = null;
        public LoggedTunableNumber fkA = null;
        
        public PIDSlot slot;

        public InternalPIDConstants(
            OptionalDouble p, OptionalDouble i, OptionalDouble d,
            OptionalDouble iZone,
            OptionalDouble fkS, OptionalDouble fkV, OptionalDouble fkA,
            PIDSlot slot
        ) {
            String slotStr = slot == PIDSlot.Slot0 ? "" : Integer.toString(slot.ordinal());
            if(p.isPresent()) this.p = new LoggedTunableNumber(tunablePath + slotStr + "/P", p.getAsDouble());
            if(i.isPresent()) this.i = new LoggedTunableNumber(tunablePath + slotStr + "/I", i.getAsDouble());
            if(d.isPresent()) this.d = new LoggedTunableNumber(tunablePath + slotStr + "/D", d.getAsDouble());
            
            if(iZone.isPresent()) this.iZone = new LoggedTunableNumber(tunablePath + slotStr + "/IZone", i.getAsDouble());

            if(fkS.isPresent()) this.fkS = new LoggedTunableNumber(tunablePath + slotStr + "/FkS", fkS.getAsDouble());
            if(fkV.isPresent()) this.fkV = new LoggedTunableNumber(tunablePath + slotStr + "/FkV", fkV.getAsDouble());
            if(fkA.isPresent()) this.fkA = new LoggedTunableNumber(tunablePath + slotStr + "/FkA", fkA.getAsDouble());
            
            this.slot = slot;

            hasChanged(); // Don't fire on initial creation
        }

        public boolean hasChanged() {
            return (p != null && p.hasChanged(hashCode())) ||
                (i != null && i.hasChanged(hashCode())) ||
                (d != null && d.hasChanged(hashCode())) ||
                (fkV != null && fkV.hasChanged(hashCode())) ||
                (iZone != null && iZone.hasChanged(hashCode()));
        }
    }

    /** The set of PID slots used for a real robot; only used to appropriately copy to sim. */
    private ArrayList<GenericPIDConstants> realPidGains;
    /** The set of PID slots to be used. */
    private ArrayList<InternalPIDConstants> pidSlots;
    /** The path to the tunable constants. */
    private String tunablePath;
    /** The list of sparks to be configured. */
    private HashSet<SparkBase> sparks = new HashSet<>();
    /** The list of talons to be configured. */
    private HashSet<TalonFX> talons = new HashSet<>();

    private static record ConfigurableController(PIDController controller, PIDSlot slot) {};
    /** The list of normal PID controllers to be configured. */
    private HashSet<ConfigurableController> controllers = new HashSet<>();

    /** A list of change listeners that are run every loop iteration when in tuning mode. */
    private static ArrayList<Runnable> changeListenerRegistry = new ArrayList<>();

    /**
     * A static method to be called every loop iteration to check for changes and re-configure the sparks. Doesn't
     * unnecessarily run if not in tuning mode.
     */
    public static void periodic() {
        if(!Robot.tuningMode()) { return; }
        for(Runnable r : changeListenerRegistry) {
            r.run();
        }
    }

    /**
     * Creates a new TunableSparkPID object with the given path. The path is used to create the tunable numbers for the
     * PID constants. Note that this object will never be cleaned up once created, so it should be a constant.
     *
     * @param tunablePath The path to the tunable constants.
     */
    public TunablePID(String tunablePath) {
        realPidGains = new ArrayList<>();
        pidSlots = new ArrayList<>();
        this.tunablePath = tunablePath;

        // Register a change listener to check for changes
        changeListenerRegistry.add(this::checkChange);
    }

    /**
     * Gets the closed loop configuration for the current mode. This should be used with
     * `SparkMaxConfiguration.closedLoop.apply` or `SparkFlexConfiguration.closedLoop.apply` when configuring the motor
     * controllers.
     * @return
     */
    public ClosedLoopConfig getSparkConfig() {
        ClosedLoopConfig config = new ClosedLoopConfig();
        for(InternalPIDConstants c : pidSlots) {
            config.p(c.p == null ? 0. : c.p.get(), c.slot.rev);
            config.i(c.i == null ? 0. : c.i.get(), c.slot.rev);
            config.d(c.d == null ? 0. : c.d.get(), c.slot.rev);
            config.feedForward.kV(c.fkV == null ? 0. : c.fkV.get(), c.slot.rev);
            config.feedForward.kA(c.fkA == null ? 0. : c.fkA.get(), c.slot.rev);
            config.feedForward.kS(c.fkS == null ? 0. : c.fkS.get(), c.slot.rev);
            if(c.iZone != null) config.iZone(c.iZone.get(), c.slot.rev);
        }
        return config;
    }

    public boolean hasSlotConfigured(PIDSlot slot) {
        for(InternalPIDConstants c : pidSlots) {
            if(c.slot == slot) return true;
        }
        return false;
    }

    /**
     * Get the Talon slot configs for the current mode. This should be used when configuring Talon FXs.
     * @param config
     * @param sparks
     */
    public SlotConfigs getTalonSlotConfigs(PIDSlot slot) {
        SlotConfigs configs = new SlotConfigs();
        configs.SlotNumber = slot.ordinal();
        for(InternalPIDConstants c : pidSlots) {
            if(c.slot == slot) {
                configs.kP = c.p == null ? 0. : c.p.get();
                configs.kI = c.i == null ? 0. : c.i.get();
                configs.kD = c.d == null ? 0. : c.d.get();
                configs.kV = c.fkV == null ? 0. : c.fkV.get();
                // izone not supported
            }
        }
        return configs;
    }

    /**
     * Applies the PID configuration to the given spark(s) and adds it to be reconfigured when the PID
     * constants change.
     * @param config
     * @param sparks
     */
    public void applyConfigAndRegister(SparkBaseConfig config, SparkBase... sparks) {
        config.closedLoop.apply(getSparkConfig());
        for(SparkBase spark : sparks) {
            configureSparkOnChange(spark);
        }
    }
    
    /**
     * Applies the PID configuration to the given Talon FX(s) and adds it to be reconfigured when the PID
     * constants change.
     * @param config
     * @param sparks
     */
    public void applyConfigAndRegister(TalonFXConfiguration configuration, TalonFX... talons) {
        if(hasSlotConfigured(PIDSlot.Slot0)) configuration.Slot0 = Slot0Configs.from(getTalonSlotConfigs(PIDSlot.Slot0));
        if(hasSlotConfigured(PIDSlot.Slot1)) configuration.Slot1 = Slot1Configs.from(getTalonSlotConfigs(PIDSlot.Slot1));
        if(hasSlotConfigured(PIDSlot.Slot2)) configuration.Slot2 = Slot2Configs.from(getTalonSlotConfigs(PIDSlot.Slot2));

        for(TalonFX talon : talons) {
            configureTalonOnChange(talon);
        }
    }

    /**
     * Registers a spark to be configured when the PID constants change. This should be called for each spark that needs
     * to be configured. Note that this doesn't immediately configure the spark.
     * @param spark
     */
    public void configureSparkOnChange(SparkBase spark) {
        if(!Robot.tuningMode()) { return; } // No need to do unnecessary work if not in tuning mode
        sparks.add(spark);
    }
    
    /**
     * Registers a Talon FX to be configured when the PID constants change. This should be called for each Talon FX that needs
     * to be configured. Note that this doesn't immediately configure the Talon FX.
     */
    public void configureTalonOnChange(TalonFX talon) {
        if(!Robot.tuningMode()) { return; } // No need to do unnecessary work if not in tuning mode
        talons.add(talon);
    }

    /**
     * Registers a normal PIDController to be configured when the PID constants change. This is only really useful for
     * simulation when we want to bypass REV's control loops.
     * @param controller
     * @param slot
     */
    public void configureController(PIDController controller, PIDSlot slot) {
        controllers.add(new ConfigurableController(controller, slot));
        for(InternalPIDConstants c : pidSlots) {
            if(c.slot == slot) setControllerConfig(controller, c);
        }
    }

    private void setControllerConfig(PIDController controller, InternalPIDConstants constants) {
        controller.setP(constants.p == null ? 0. : constants.p.get());
        controller.setI(constants.i == null ? 0. : constants.i.get());
        controller.setD(constants.d == null ? 0. : constants.d.get());
    }

    /**
     * Checks if any of the PID constants have changed. If they have, it reconfigures all of the sparks that have been
     * registered. This is called every loop iteration when in tuning mode.
     */
    private void checkChange() {
        for(InternalPIDConstants c : pidSlots) {
            if(c.hasChanged()) {
                for(SparkBase spark : sparks) {
                    boolean isFlex = spark instanceof SparkFlex;
                    SparkBaseConfig config = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
                    config.closedLoop.apply(getSparkConfig());

                    tryUntilOk(spark, 5, () -> spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
                }

                for(TalonFX talon : talons) {
                    if(hasSlotConfigured(PIDSlot.Slot0)) PhoenixUtil.tryUntilOk(5,
                        () -> talon.getConfigurator().apply(getTalonSlotConfigs(PIDSlot.Slot0))
                    );
                    if(hasSlotConfigured(PIDSlot.Slot1)) PhoenixUtil.tryUntilOk(5,
                        () -> talon.getConfigurator().apply(getTalonSlotConfigs(PIDSlot.Slot1))
                    );
                    if(hasSlotConfigured(PIDSlot.Slot2)) PhoenixUtil.tryUntilOk(5,
                        () -> talon.getConfigurator().apply(getTalonSlotConfigs(PIDSlot.Slot2))
                    );
                }

                for(ConfigurableController cc : controllers) {
                    if(c.slot == cc.slot) setControllerConfig(cc.controller, c);
                }

                var total = sparks.size() + talons.size() + controllers.size();
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Tunable PIDs", "Configured " + total + " controllers with updated PIDs!"));
                return;
            }
        }
    }

    // A bunch of different overloads so the feedforward and slot are optional

    /**
     * Adds a new set of PID constants to the list of constants for the real robot.
     * @param p
     * @param i
     * @param d
     * @param fkV
     * @param slot
     */
    public TunablePID addRealRobotGains(GenericPIDConstants constants) {
        realPidGains.add(constants);
        if(Constants.currentMode == Constants.Mode.REAL) pidSlots.add(constants.toInternal(this));
        return this;
    }

    /**
     * Adds a new set of PID constants to the list of constants for the simulated robot.
     * @param p
     * @param i
     * @param d
     * @param fkV
     * @param slot
     */
    public TunablePID addSimGains(GenericPIDConstants constants) {
        if(Constants.currentMode != Constants.Mode.REAL) pidSlots.add(constants.toInternal(this));
        return this;
    }

    /**
     * Copies the real robot gains to simulation.
     */
    public TunablePID copyRealGainsInSim() {
        if(Constants.currentMode != Constants.Mode.REAL) {
            for(GenericPIDConstants c : realPidGains) {
                pidSlots.add(c.toInternal(this));
            }
        }
        return this;
    }
}
