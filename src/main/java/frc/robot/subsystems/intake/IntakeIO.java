// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
  public static class IntakeIOInputs {

    public double intakeStatorCurrent = 0.0;
    public double intakeSupplyCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public double intakeMotorTemp = 0.0;
    public boolean intakeMotorConnected = false;
    public boolean intakeEncoderConnected = false;
    public double intakeSpeed = 0.0;

  }
  
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOutput(double voltage) {}
}
