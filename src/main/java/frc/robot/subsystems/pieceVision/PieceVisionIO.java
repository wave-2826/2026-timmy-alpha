package frc.robot.subsystems.pieceVision;

import org.littletonrobotics.junction.AutoLog;

public interface PieceVisionIO {
    @AutoLog
    public static class PieceVisionIOInputs {
        public boolean connected = false;
    }

    default void updateInputs(PieceVisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
