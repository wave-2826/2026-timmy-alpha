package frc.robot.util;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.FieldConstants;

import java.util.Arrays;

/**
 * Collection of different feedforward values for each drive module. If using swerve, these values
 * will all be in FL, FR, BL, BR order. If using a differential drive, these will be in L, R order.
 *
 * <p>NOTE: If using Choreo paths, all feedforwards but the X and Y component arrays will be filled
 * with zeros.
 *
 * @param accelerationsMPSSq Linear acceleration at the wheels in meters per second
 * @param linearForcesNewtons Linear force applied by the motors at the wheels in newtons
 * @param torqueCurrentsAmps Torque-current of the drive motors in amps
 * @param robotRelativeForcesXNewtons X components of robot-relative force vectors for the wheels in
 *     newtons. The magnitude of these vectors will typically be greater than the linear force
 *     feedforwards due to friction forces.
 * @param robotRelativeForcesYNewtons X components of robot-relative force vectors for the wheels in
 *     newtons. The magnitude of these vectors will typically be greater than the linear force
 *     feedforwards due to friction forces.
 */
public record DriveFeedforwards(
    double[] accelerationsMPSSq,
    double[] linearForcesNewtons,
    double[] torqueCurrentsAmps,
    double[] robotRelativeForcesXNewtons,
    double[] robotRelativeForcesYNewtons)
    implements Interpolatable<DriveFeedforwards> {
  /**
   * Collection of different feedforward values for each drive module. If using swerve, these values
   * will all be in FL, FR, BL, BR order. If using a differential drive, these will be in L, R
   * order.
   *
   * <p>NOTE: If using Choreo paths, all feedforwards but the X and Y component arrays will be
   * filled with zeros.
   *
   * @param accelerations Linear acceleration at the wheels
   * @param linearForces Linear force applied by the motors at the wheels
   * @param torqueCurrents Torque-current of the drive motors
   * @param robotRelativeForcesX X components of robot-relative force vectors for the wheels. The
   *     magnitude of these vectors will typically be greater than the linear force feedforwards due
   *     to friction forces.
   * @param robotRelativeForcesY X components of robot-relative force vectors for the wheels. The
   *     magnitude of these vectors will typically be greater than the linear force feedforwards due
   *     to friction forces.
   */
  public DriveFeedforwards(
      LinearAcceleration[] accelerations,
      Force[] linearForces,
      Current[] torqueCurrents,
      Force[] robotRelativeForcesX,
      Force[] robotRelativeForcesY) {
    this(
        Arrays.stream(accelerations).mapToDouble(x -> x.in(MetersPerSecondPerSecond)).toArray(),
        Arrays.stream(linearForces).mapToDouble(x -> x.in(Newtons)).toArray(),
        Arrays.stream(torqueCurrents).mapToDouble(x -> x.in(Amps)).toArray(),
        Arrays.stream(robotRelativeForcesX).mapToDouble(x -> x.in(Newtons)).toArray(),
        Arrays.stream(robotRelativeForcesY).mapToDouble(x -> x.in(Newtons)).toArray());
  }

  

  /**
   * Create drive feedforwards consisting of all zeros
   *
   * @param numModules Number of drive modules
   * @return Zero feedforwards
   */
  public static DriveFeedforwards zeros(int numModules) {
    return new DriveFeedforwards(
        new double[numModules],
        new double[numModules],
        new double[numModules],
        new double[numModules],
        new double[numModules]);
  }

  @Override
  public DriveFeedforwards interpolate(DriveFeedforwards endValue, double t) {
    return new DriveFeedforwards(
        interpolateArray(accelerationsMPSSq, endValue.accelerationsMPSSq, t),
        interpolateArray(linearForcesNewtons, endValue.linearForcesNewtons, t),
        interpolateArray(torqueCurrentsAmps, endValue.torqueCurrentsAmps, t),
        interpolateArray(robotRelativeForcesXNewtons, endValue.robotRelativeForcesXNewtons, t),
        interpolateArray(robotRelativeForcesYNewtons, endValue.robotRelativeForcesYNewtons, t));
  }

  /**
   * Reverse the feedforwards for driving backwards. This should only be used for differential drive
   * robots.
   *
   * @return Reversed feedforwards
   */
  public DriveFeedforwards reverse() {
    if (accelerationsMPSSq.length != 2) {
      throw new IllegalStateException(
          "Feedforwards should only be reversed for differential drive trains");
    }

    return new DriveFeedforwards(
        new double[] {-accelerationsMPSSq[1], -accelerationsMPSSq[0]},
        new double[] {-linearForcesNewtons[1], -linearForcesNewtons[0]},
        new double[] {-torqueCurrentsAmps[1], -torqueCurrentsAmps[0]},
        new double[] {-robotRelativeForcesXNewtons[1], -robotRelativeForcesXNewtons[0]},
        new double[] {-robotRelativeForcesYNewtons[1], -robotRelativeForcesYNewtons[0]});
  }

  /**
   * Flip the feedforwards for the other side of the field. Only does anything if mirrored symmetry
   * is used
   *
   * @return Flipped feedforwards
   */
  public DriveFeedforwards flip() {
    return new DriveFeedforwards(
        FlippingUtil.flipFeedforwards(accelerationsMPSSq),
        FlippingUtil.flipFeedforwards(linearForcesNewtons),
        FlippingUtil.flipFeedforwards(torqueCurrentsAmps),
        FlippingUtil.flipFeedforwardXs(robotRelativeForcesXNewtons),
        FlippingUtil.flipFeedforwardYs(robotRelativeForcesYNewtons));
  }

  /**
   * Get the linear accelerations at the wheels
   *
   * @return Linear accelerations at the wheels
   */
  public LinearAcceleration[] accelerations() {
    return Arrays.stream(accelerationsMPSSq)
        .mapToObj(MetersPerSecondPerSecond::of)
        .toArray(LinearAcceleration[]::new);
  }

  /**
   * Get the linear forces at the wheels
   *
   * @return Linear forces at the wheels
   */
  public Force[] linearForces() {
    return Arrays.stream(linearForcesNewtons).mapToObj(Newtons::of).toArray(Force[]::new);
  }

  /**
   * Get the torque-current of the drive motors
   *
   * @return Torque-current of the drive motors
   */
  public Current[] torqueCurrents() {
    return Arrays.stream(torqueCurrentsAmps).mapToObj(Amps::of).toArray(Current[]::new);
  }

  /**
   * Get the X components of the robot-relative force vectors at the wheels
   *
   * @return X components of the robot-relative force vectors at the wheels
   */
  public Force[] robotRelativeForcesX() {
    return Arrays.stream(robotRelativeForcesXNewtons).mapToObj(Newtons::of).toArray(Force[]::new);
  }

  /**
   * Get the Y components of the robot-relative force vectors at the wheels
   *
   * @return Y components of the robot-relative force vectors at the wheels
   */
  public Force[] robotRelativeForcesY() {
    return Arrays.stream(robotRelativeForcesYNewtons).mapToObj(Newtons::of).toArray(Force[]::new);
  }

  private static double[] interpolateArray(double[] a, double[] b, double t) {
    double[] ret = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      ret[i] = MathUtil.interpolate(a[i], b[i], t);
    }
    return ret;
  }

  /** Utility class for flipping positions/rotations to the other side of the field */
  public class FlippingUtil {
  /** The type of symmetry for the current field */
  public static FieldSymmetry symmetryType = FieldSymmetry.kRotational;
  /** The X size or length of the current field in meters */
  public static double fieldSizeX = FieldConstants.fieldLengthX;
  /** The Y size or width of the current field in meters */
  public static double fieldSizeY = FieldConstants.fieldWidthY;

  /** Enum representing the different types of field symmetry */
  public enum FieldSymmetry {
    /**
     * Field is rotationally symmetric. i.e. the red alliance side is the blue alliance side rotated
     * by 180 degrees
     */
    kRotational,
    /** Field is mirrored vertically over the center of the field */
    kMirrored
  }


    /**
   * Flip an array of drive feedforwards for the other side of the field. Only does anything if
   * mirrored symmetry is used
   *
   * @param feedforwards Array of drive feedforwards
   * @return The flipped feedforwards
   */
  public static double[] flipFeedforwards(double[] feedforwards) {
    return switch (symmetryType) {
      case kMirrored -> {
        if (feedforwards.length == 4) {
          yield new double[] {feedforwards[1], feedforwards[0], feedforwards[3], feedforwards[2]};
        } else if (feedforwards.length == 2) {
          yield new double[] {feedforwards[1], feedforwards[0]};
        }
        yield feedforwards; // idk
      }
      case kRotational -> feedforwards;
    };
  }

  /**
   * Flip an array of drive feedforward X components for the other side of the field. Only does
   * anything if mirrored symmetry is used
   *
   * @param feedforwardXs Array of drive feedforward X components
   * @return The flipped feedforward X components
   */
  public static double[] flipFeedforwardXs(double[] feedforwardXs) {
    return flipFeedforwards(feedforwardXs);
  }

  /**
   * Flip an array of drive feedforward Y components for the other side of the field. Only does
   * anything if mirrored symmetry is used
   *
   * @param feedforwardYs Array of drive feedforward Y components
   * @return The flipped feedforward Y components
   */
  public static double[] flipFeedforwardYs(double[] feedforwardYs) {
    var flippedFeedforwardYs = flipFeedforwards(feedforwardYs);
    return switch (symmetryType) {
      case kMirrored -> {
        // Y directions also need to be inverted
        for (int i = 0; i < flippedFeedforwardYs.length; ++i) {
          flippedFeedforwardYs[i] *= -1;
        }
        yield flippedFeedforwardYs;
      }
      case kRotational -> flippedFeedforwardYs;
    };
  }
}
}