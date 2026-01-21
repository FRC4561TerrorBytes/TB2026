package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface ClimberIO {

    @AutoLog
  public static class ClimberIOInputs {
    public boolean climberConnected = false;
    public double climberPositionRad = 0.0;
    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;

}

/** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

/** Run the climber motor at a specified speed. */
  public default void setClimberVoltage(double voltage) {}

/**Set the climber to a specified position. */
  public default void setClimberPosition(double position) {}

}
