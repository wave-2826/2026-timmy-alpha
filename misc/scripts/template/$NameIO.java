package frc.robot.subsystems.$name;

import org.littletonrobotics.junction.AutoLog;

public interface $NameIO {
  @AutoLog
  public static class $NameIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs($NameIOInputs inputs) {}

  /** Run open loop at the specified power. */
  public default void setPower(double power) {}
}