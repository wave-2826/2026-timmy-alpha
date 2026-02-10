package frc.robot.util;

import java.util.OptionalDouble;

import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.util.tunables.TunableSparkPID;
import frc.robot.util.tunables.TunableSparkPID.InternalPIDConstants;

/** A set of PID constants with tunable numbers for each for logged tunable PIDs. */
public class SparkPIDConstants {
    public OptionalDouble p;
    public OptionalDouble i;
    public OptionalDouble iZone;
    public OptionalDouble d;
    public OptionalDouble f;
    public ClosedLoopSlot slot;

    public SparkPIDConstants(OptionalDouble p, OptionalDouble i, OptionalDouble iZone, OptionalDouble d, OptionalDouble f, ClosedLoopSlot slot) {
        this.p = p;
        this.i = i;
        this.i = i;
        this.iZone = iZone;
        this.d = d;
        this.f = f;
        this.slot = slot;
    }

    public SparkPIDConstants(double p, double i, double iZone, double d, double f, ClosedLoopSlot slot) {
        this(OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.of(iZone), OptionalDouble.of(d),
            OptionalDouble.of(f), slot);
    }

    public SparkPIDConstants(double p, double i, double d, double f, ClosedLoopSlot slot) {
        this(OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.empty(), OptionalDouble.of(d),
            OptionalDouble.of(f), slot);
    }

    public SparkPIDConstants(double p, double i, double d, ClosedLoopSlot slot) {
        this(OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.empty(), OptionalDouble.of(d),
            OptionalDouble.empty(), slot);
    }

    public SparkPIDConstants(double p, double i, double iZone, double d, double f) {
        this(p, i, iZone, d, f, ClosedLoopSlot.kSlot0);
    }

    public SparkPIDConstants(double p, double i, double d, double f) {
        this(p, i, d, f, ClosedLoopSlot.kSlot0);
    }

    public SparkPIDConstants(double p, double i, double d) {
        this(p, i, d, ClosedLoopSlot.kSlot0);
    }

    public SparkPIDConstants iZone(double iZone) {
        this.iZone = OptionalDouble.of(iZone);
        return this;
    }

    public InternalPIDConstants toInternal(TunableSparkPID tunablePID) {
        return tunablePID.new InternalPIDConstants(this.p, this.i, this.iZone, this.d, this.f, slot);
    }
}