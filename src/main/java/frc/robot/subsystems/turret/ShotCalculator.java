package frc.robot.subsystems.turret;

public class ShotCalculator {
    private static ShotCalculator instance = null;
    public static ShotCalculator getInstance() {
        if(instance == null) {
            instance = new ShotCalculator();
        }
        return instance;
    }
}