package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface ClimberIO {

    @AutoLog
  public static class ClimberIOInputs {
    public boolean climberConnected = false;
    public double climberVelocity = 0.0;
    public double climberVoltage = 0.0;
    public double climberCurrent = 0.0;
    public double climberPosition = 0.0;
    public double climberSetpoint = 0.0;
}

/** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

/** Run the climber motor at a specified speed. */
  public default void setClimberVoltage(double voltage) {}

/**Set the climber to a specified position. */
  public default void setClimberPosition(double position) {}

  /**Set the climber motor to coast mode */
  public default void setIdleMode(NeutralModeValue idleMode) {}
}
