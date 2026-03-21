package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotState;

public class TurretVisualizer {
    private static TurretVisualizer instance = null;
    public static TurretVisualizer getInstance() {
        if(instance == null) instance = new TurretVisualizer();
        return instance;
    }

    private LoggedMechanism2d azimuthMechanism;
    private LoggedMechanism2d hoodMechanism;

    private LoggedMechanismLigament2d azimuthTargetDisplay;
    private LoggedMechanismLigament2d azimuthMeasuredDisplay;

    private LoggedMechanismLigament2d hoodTargetDisplay;
    private LoggedMechanismLigament2d hoodMeasuredDisplay;

    private TurretVisualizer() {
        azimuthMechanism = new LoggedMechanism2d(0.8, 0.8);
        var azimuthRoot = azimuthMechanism.getRoot("turret", 0.4, 0.4);

        hoodMechanism = new LoggedMechanism2d(0.8, 0.8);
        var hoodRoot = hoodMechanism.getRoot("azimuth", 0.05, 0.05);

        azimuthTargetDisplay = new LoggedMechanismLigament2d("azimuthTarget", 0.3, 0);
        azimuthTargetDisplay.setColor(new Color8Bit(Color.kPink));

        azimuthMeasuredDisplay = new LoggedMechanismLigament2d("azimuthMeasured", 0.3, 0);
        azimuthMeasuredDisplay.setColor(new Color8Bit(Color.kRed));

        azimuthRoot.append(azimuthTargetDisplay);
        azimuthRoot.append(azimuthMeasuredDisplay);

        hoodTargetDisplay = new LoggedMechanismLigament2d("hoodTarget", 0.3, 0);
        hoodTargetDisplay.setColor(new Color8Bit(Color.kLightBlue));

        hoodMeasuredDisplay = new LoggedMechanismLigament2d("hoodMeasured", 0.3, 0);
        hoodMeasuredDisplay.setColor(new Color8Bit(Color.kBlue));

        hoodRoot.append(hoodTargetDisplay);
        hoodRoot.append(hoodMeasuredDisplay);
    }

    public void update(
        double targetAzimuthRad, double measuredAzimuthRad,
        double targetHoodRad, double measuredHoodRad
    ) {
        azimuthTargetDisplay.setAngle(Units.radiansToDegrees(targetAzimuthRad));
        azimuthMeasuredDisplay.setAngle(Units.radiansToDegrees(measuredAzimuthRad));

        hoodTargetDisplay.setAngle(Units.radiansToDegrees(targetHoodRad));
        hoodMeasuredDisplay.setAngle(Units.radiansToDegrees(measuredHoodRad));

        var targetBallPathPitch = targetHoodRad + Math.PI;
        var measuredBallPathPitch = measuredHoodRad + Math.PI;

        var startPos = new Pose3d(RobotState.getInstance().getBestEstimatedPose())
            .getTranslation().plus(TurretConstants.robotToTurret);
        var endPosTarget = startPos.plus(new Translation3d(1, 0, 0).rotateBy(new Rotation3d(
            0,
            targetBallPathPitch,
            targetAzimuthRad
        )));
        var endPosMeasured = startPos.plus(new Translation3d(1, 0, 0).rotateBy(new Rotation3d(
            0,
            measuredBallPathPitch,
            measuredAzimuthRad
        )));

        Logger.recordOutput("Turret/Mechanism/Azimuth", azimuthMechanism);
        Logger.recordOutput("Turret/Mechanism/Hood", hoodMechanism);
        Logger.recordOutput("Turret/Mechanism/LinesTarget", new Translation3d[][] {
            new Translation3d[] { startPos, endPosTarget },
        });
        Logger.recordOutput("Turret/Mechanism/LinesMeasured", new Translation3d[][] {
            new Translation3d[] { startPos, endPosMeasured },
        });
    }
}
