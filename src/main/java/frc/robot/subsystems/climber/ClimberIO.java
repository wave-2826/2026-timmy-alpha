package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public record ClimberMotorInputs(
      /** Whether the motor is connected */
      boolean connected,
      /** The motor current draw. */
      double currentAmps,
      /** The motor's position, according to the encoder, in meters */
      double position
    ) {}

    ClimberMotorInputs right = new ClimberMotorInputs(false, 0, 0);
    ClimberMotorInputs left = new ClimberMotorInputs(false, 0, 0);
    
    double rightServeoPosition = 0;
    double leftServeoPosition = 0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Run open loop at the specified power. */
  public default void setLeftPower(double power) {}
  public default void setRightPower(double power) {}

  /** Sets the servo's position */
  public default void setLeftServoPosition(double position) {}
  /** Sets the servo's position */
  public default void setRightServoPosition(double position) {}
}