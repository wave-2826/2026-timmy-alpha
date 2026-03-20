package frc.robot.util;

import java.util.OptionalDouble;

import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.util.tunables.TunablePID;
import frc.robot.util.tunables.TunablePID.InternalPIDConstants;

/** A set of PID constants with tunable numbers for each for logged tunable PIDs. */
public class GenericPIDConstants {
    public enum PIDSlot {
        Slot0(ClosedLoopSlot.kSlot0),
        Slot1(ClosedLoopSlot.kSlot1),
        Slot2(ClosedLoopSlot.kSlot2),
        Slot3(ClosedLoopSlot.kSlot3);

        public final ClosedLoopSlot rev;
        // public final Slot
        private PIDSlot(ClosedLoopSlot revSlot) {
            this.rev = revSlot;
        }
    }
    
    public OptionalDouble p;
    public OptionalDouble i;
    public OptionalDouble d;

    public OptionalDouble iZone;
    
    public OptionalDouble fkS;
    public OptionalDouble fkV;
    public OptionalDouble fkA;
    
    public PIDSlot slot;

    public GenericPIDConstants(
        OptionalDouble p, OptionalDouble i, OptionalDouble d, OptionalDouble fkV,
        OptionalDouble iZone,
        OptionalDouble fkS, OptionalDouble fkA,
        PIDSlot slot) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.iZone = iZone;
        
        this.fkS = fkS;
        this.fkV = fkV;
        this.fkA = fkA;
        
        this.slot = slot;
    }

    public GenericPIDConstants(double p, double i, double d, double f, PIDSlot slot) {
        this(
            OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.of(d),
            OptionalDouble.of(f),
            OptionalDouble.empty(), OptionalDouble.empty(), OptionalDouble.empty(),
            slot
        );
    }

    public GenericPIDConstants(double p, double i, double d, PIDSlot slot) {
        this(
            OptionalDouble.of(p), OptionalDouble.of(i), OptionalDouble.of(d),
            OptionalDouble.empty(),
            OptionalDouble.empty(), OptionalDouble.empty(), OptionalDouble.empty(),
            slot
        );
    }
    public GenericPIDConstants(double p, double i, double d, double f) {
        this(p, i, d, f, PIDSlot.Slot0);
    }

    public GenericPIDConstants(double p, double i, double d) {
        this(p, i, d, PIDSlot.Slot0);
    }

    public GenericPIDConstants iZone(double iZone) {
        this.iZone = OptionalDouble.of(iZone);
        return this;
    }

    public GenericPIDConstants kS(double fkS) {
        this.fkS = OptionalDouble.of(fkS);
        return this;
    }
    public GenericPIDConstants kV(double fkV) {
        this.fkV = OptionalDouble.of(fkV);
        return this;
    }
    public GenericPIDConstants kA(double fkA) {
        this.fkA = OptionalDouble.of(fkA);
        return this;
    }

    public GenericPIDConstants sva(double fkS, double fkV, double fkA) {
        return kS(fkS).kV(fkV).kA(fkA);
    }

    public GenericPIDConstants slot(PIDSlot slot) {
        this.slot = slot;
        return this;
    }

    public InternalPIDConstants toInternal(TunablePID tunablePID) {
        return tunablePID.new InternalPIDConstants(
            p, i, d,
            iZone,
            fkS, fkV, fkA,
            slot
        );
    }
}