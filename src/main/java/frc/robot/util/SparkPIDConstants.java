package frc.robot.util;

import java.util.OptionalDouble;

import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.util.tunables.TunableSparkPID;
import frc.robot.util.tunables.TunableSparkPID.InternalPIDConstants;

/** A set of PID constants with tunable numbers for each for logged tunable PIDs. */
public class SparkPIDConstants {
    public OptionalDouble p;
    public OptionalDouble i;
    public OptionalDouble d;

    public OptionalDouble iZone;
    
    public OptionalDouble fkS;
    public OptionalDouble fkV;
    public OptionalDouble fkA;
    
    public ClosedLoopSlot slot;

    public SparkPIDConstants(
        OptionalDouble p, OptionalDouble i, OptionalDouble d, OptionalDouble fkV,
        OptionalDouble iZone,
        OptionalDouble fkS, OptionalDouble fkA,
        ClosedLoopSlot slot) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.iZone = iZone;
        
        this.fkS = fkS;
        this.fkV = fkV;
        this.fkA = fkA;
        
        this.slot = slot;
    }

    public SparkPIDConstants(double p, double i, double d, double f, ClosedLoopSlot slot) {
        this(
            OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.of(d),
            OptionalDouble.of(f),
            OptionalDouble.empty(), OptionalDouble.empty(), OptionalDouble.empty(),
            slot
        );
    }

    public SparkPIDConstants(double p, double i, double d, ClosedLoopSlot slot) {
        this(
            OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.of(d),
            OptionalDouble.empty(),
            OptionalDouble.empty(), OptionalDouble.empty(), OptionalDouble.empty(),
            slot
        );
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

    public SparkPIDConstants kS(double fkS) {
        this.fkS = OptionalDouble.of(fkS);
        return this;
    }
    public SparkPIDConstants kV(double fkV) {
        this.fkV = OptionalDouble.of(fkV);
        return this;
    }
    public SparkPIDConstants kA(double fkA) {
        this.fkA = OptionalDouble.of(fkA);
        return this;
    }

    public SparkPIDConstants sva(double fkS, double fkV, double fkA) {
        return kS(fkS).kV(fkV).kA(fkA);
    }

    public SparkPIDConstants slot(ClosedLoopSlot slot) {
        this.slot = slot;
        return this;
    }

    public InternalPIDConstants toInternal(TunableSparkPID tunablePID) {
        return tunablePID.new InternalPIDConstants(
            p, i, d,
            iZone,
            fkS, fkV, fkA,
            slot
        );
    }
}