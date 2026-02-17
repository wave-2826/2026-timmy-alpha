package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public record climbMotorInputs(
      /** Whether the motor is connected */
      boolean connected,
      /** The motor current draw. */
      double motorCurrentAmps,
      /** The motor's position according to the encoder in meters */
      double motorPosition
    ) {}
    climbMotorInputs right = new climbMotorInputs(false, 0, 0);
    climbMotorInputs left = new climbMotorInputs(false, 0, 0);
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run open loop at the specified power. */
  public default void setLeftPower(double power) {}
  public default void setRightPower(double power) {}
}