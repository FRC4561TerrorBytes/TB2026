// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
  public static class IntakeIOInputs {

    public double intakeLeftStatorCurrent = 0.0;
    public double intakeLeftSupplyCurrent = 0.0;
    public double intakeLeftVoltage = 0.0;
    public double intakeLeftMotorTemp = 0.0;
    public boolean intakeLeftMotorConnected = false;
    public double intakeLeftSpeed = 0.0;

    public double intakeRightStatorCurrent = 0.0;
    public double intakeRightSupplyCurrent = 0.0;
    public double intakeRightVoltage = 0.0;
    public double intakeRightMotorTemp = 0.0;
    public boolean intakeRightMotorConnected = false;
    public double intakeRightSpeed = 0.0;
  }
  
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOutput(double voltage) {}
}
