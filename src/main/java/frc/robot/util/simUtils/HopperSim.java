package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

public class HopperSim {
    private int maxFuel = 50;
    private int fuelInHopper = 0;

    private static double hopperSizeX = DriveConstants.wheelBaseX.in(Meters);
    private static double hopperSizeY = DriveConstants.trackWidthY.in(Meters);
    private static double hopperSizeZ = Units.inchesToMeters(20);
    private static double hopperCenterZ = Units.inchesToMeters(15);

    public boolean canIntake() {
        return fuelInHopper <= maxFuel;
    }

    public Translation3d[] getHopperFuelFieldPositions(Pose3d robotPose) {
        // Basic packing based on hopper size and max fuel
        Translation3d[] positions = new Translation3d[fuelInHopper];
        
        for(int i = 0; i < fuelInHopper; i++) {
            double x = (i % 5) * (hopperSizeX / 5) - hopperSizeX / 2 + hopperSizeX / 10;
            double y = ((i / 5) % 5) * (hopperSizeY / 5) - hopperSizeY / 2 + hopperSizeY / 10;
            double z = (i / 25) * (hopperSizeZ / 4) + hopperCenterZ - hopperSizeZ / 2 + hopperSizeZ / 8;
            positions[i] = robotPose.plus(
                new Transform3d(new Translation3d(x, y, z), Rotation3d.kZero)
            ).getTranslation();
        }

        return positions;
    }

    public void addFuel() {
        if(fuelInHopper >= maxFuel) {
            System.out.println("HopperSim: tried to intake fuel with no more space");
            return;
        }

        fuelInHopper += 1;
    }

    public boolean removeFuel() {
        if(fuelInHopper <= 0) {
            return false;
        }

        fuelInHopper -= 1;
        return true;
    }
}
