package frc.robot.subsystems.hopperVision;

import org.littletonrobotics.junction.AutoLog;

public interface HopperVisionIO {
    @AutoLog
    public static class HopperVisionIOInputs {
        public boolean connected = false;
        public int targets = 0;
        public int fpsLimit = 0;
    }
    
    default void updateInputs(HopperVisionIOInputs inputs) {}
    default void setFPSLimit(int fps) {}
}
