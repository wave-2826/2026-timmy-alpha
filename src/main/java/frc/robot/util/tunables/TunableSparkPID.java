package frc.robot.util.tunables;

import java.util.ArrayList;
import java.util.OptionalDouble;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.tryUntilOk;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SparkPIDConstants;

public class TunableSparkPID {
    /** A set of PID constants with tunable numbers for each for logged tunable PIDs. */
    public class InternalPIDConstants {
        public LoggedTunableNumber p = null;
        public LoggedTunableNumber i = null;
        public LoggedTunableNumber iZone = null;
        public LoggedTunableNumber d = null;
        public LoggedTunableNumber f = null;
        public ClosedLoopSlot slot;

        public InternalPIDConstants(OptionalDouble p, OptionalDouble i, OptionalDouble iZone, OptionalDouble d,
            OptionalDouble f, ClosedLoopSlot slot) {
            if(p.isPresent()) this.p = new LoggedTunableNumber(tunablePath + slot.toString() + "P", p.getAsDouble());
            if(i.isPresent()) this.i = new LoggedTunableNumber(tunablePath + slot.toString() + "I", i.getAsDouble());
            if(i.isPresent()) this.i = new LoggedTunableNumber(tunablePath + slot.toString() + "IZone",
                i.getAsDouble());
            if(iZone.isPresent()) this.iZone = new LoggedTunableNumber(tunablePath + slot.toString() + "IZone",
                i.getAsDouble());
            if(d.isPresent()) this.d = new LoggedTunableNumber(tunablePath + slot.toString() + "D", d.getAsDouble());
            if(f.isPresent()) this.f = new LoggedTunableNumber(tunablePath + slot.toString() + "F", f.getAsDouble());
            this.slot = slot;

            hasChanged(); // Don't fire on initial creation
        }

        public boolean hasChanged() {
            return (p != null && p.hasChanged(hashCode())) || //
                (i != null && i.hasChanged(hashCode())) || //
                (d != null && d.hasChanged(hashCode())) || //
                (f != null && f.hasChanged(hashCode())) || //
                (iZone != null && iZone.hasChanged(hashCode()));
        }
    }

    /** The set of PID slots to be used when on a simulated robot. */
    private ArrayList<InternalPIDConstants> pidSlots;
    /** The path to the tunable constants. */
    private String tunablePath;
    /** The list of sparks to be configured. */
    private ArrayList<SparkBase> sparks = new ArrayList<>();

    /** A list of change listeners that are run y loop iteration when in tuning mode. */
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
    public TunableSparkPID(String tunablePath) {
        pidSlots = new ArrayList<>();
        this.tunablePath = tunablePath;

        // Register a change listener to check for changes
        changeListenerRegistry.add(() -> {
            checkChange();
        });
    }

    /**
     * Gets the closed loop configuration for the current mode. This should be used with
     * `SparkMaxConfiguration.closedLoop.apply` or `SparkFlexConfiguration.closedLoop.apply` when configuring the motor
     * controllers.
     * @return
     */
    public ClosedLoopConfig getConfig() {
        ClosedLoopConfig config = new ClosedLoopConfig();
        for(InternalPIDConstants c : pidSlots) {
            config.p(c.p == null ? 0. : c.p.get(), c.slot);
            config.i(c.i == null ? 0. : c.i.get(), c.slot);
            config.d(c.d == null ? 0. : c.d.get(), c.slot);
            config.velocityFF(c.f == null ? 0. : c.f.get(), c.slot);
            config.iZone(c.iZone == null ? 10000. : c.iZone.get(), c.slot);
        }
        return config;
    }

    /**
     * Applies the PID configuration to the given spark(s) and adds it to be reconfigured when the PID
     * constants change.
     * @param config
     * @param sparks
     */
    public void applyConfigAndRegister(SparkBaseConfig config, SparkBase... sparks) {
        config.closedLoop.apply(getConfig());
        for(SparkBase spark : sparks) {
            configureSparkOnChange(spark);
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
     * Checks if any of the PID constants have changed. If they have, it reconfigures all of the sparks that have been
     * registered. This is called every loop iteration when in tuning mode.
     */
    private void checkChange() {
        for(InternalPIDConstants c : pidSlots) {
            if(c.hasChanged()) {
                for(SparkBase spark : sparks) {
                    boolean isFlex = spark instanceof SparkFlex;
                    SparkBaseConfig config = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
                    config.closedLoop.apply(getConfig());

                    tryUntilOk(spark, 5, () -> spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
                }
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
     * @param f
     * @param slot
     */
    public TunableSparkPID addRealRobotGains(SparkPIDConstants constants) {
        if(Constants.currentMode == Constants.Mode.REAL) pidSlots.add(constants.toInternal(this));
        return this;
    }

    /**
     * Adds a new set of PID constants to the list of constants for the simulated robot.
     * @param p
     * @param i
     * @param d
     * @param f
     * @param slot
     */
    public TunableSparkPID addSimGains(SparkPIDConstants constants) {
        if(Constants.currentMode != Constants.Mode.REAL) pidSlots.add(constants.toInternal(this));
        return this;
    }
}
