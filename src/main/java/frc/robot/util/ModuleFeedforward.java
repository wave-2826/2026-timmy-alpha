package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Wheel feedforwards for a specific module. */
public class ModuleFeedforward {
    /** Field-relative feedforward force in Newtons. */
    public double forceXNewtons = 0.;
    /** Field-relative feedforward force in Newtons. */
    public double forceYNewtons = 0.;
    
    /**
     * Constructs a module feedforward from X and Y forces relative to the field.  
     * If using choreo, moduleForcesX() and moduleForcesY() return field-relative force.
     * @param forceXNewtons
     * @param forceYNewtons
     */
    public ModuleFeedforward(double forceXNewtons, double forceYNewtons) {
        this.forceXNewtons = forceXNewtons;
        this.forceYNewtons = forceYNewtons;
    }

    public double getCosineScaledForceN(Rotation2d moduleRotation, Rotation2d moduleTarget, Rotation2d robotRotation) {
        // Transform the feedforward force into module-relative and take the X component -
        // no need to cosine scale because it's already component-wise
        Translation2d force = new Translation2d(forceXNewtons, forceYNewtons).rotateBy(moduleRotation.plus(robotRotation).unaryMinus());
        
        double targetCosineScaling = moduleTarget.minus(moduleRotation).getCos();
        return force.getX() * targetCosineScaling;
    }
}
