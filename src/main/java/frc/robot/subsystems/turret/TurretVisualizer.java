package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretVisualizer {
    private static TurretVisualizer instance = null;
    public static TurretVisualizer getInstance() {
        if(instance == null) instance = new TurretVisualizer();
        return instance;
    }

    private LoggedMechanism2d azimuth;
    private LoggedMechanismLigament2d targetDisplay;
    private LoggedMechanismLigament2d measuredDisplay;

    private TurretVisualizer() {
        azimuth = new LoggedMechanism2d(0.8, 0.8);
        var root = azimuth.getRoot("red", 0.4, 0.4);

        targetDisplay = new LoggedMechanismLigament2d("target", 0.3, 0);
        targetDisplay.setColor(new Color8Bit(Color.kBlue));

        measuredDisplay = new LoggedMechanismLigament2d("measured", 0.3, 0);
        measuredDisplay.setColor(new Color8Bit(Color.kRed));

        root.append(targetDisplay);
        root.append(measuredDisplay);
    }

    public void update(double targetAzimuthRad, double measuredAzimuthRad) {
        targetDisplay.setAngle(Units.radiansToDegrees(targetAzimuthRad));
        measuredDisplay.setAngle(Units.radiansToDegrees(measuredAzimuthRad));

        Logger.recordOutput("Turret/Mechanism", azimuth);
    }
}
