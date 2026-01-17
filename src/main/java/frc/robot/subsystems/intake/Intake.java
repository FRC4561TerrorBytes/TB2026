// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert IntakeDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    IntakeDisconnectedAlert = new Alert("Intake motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Io", inputs);
    IntakeDisconnectedAlert.set(!inputs.IntakeMotorConnected);
  }
  
  public void setExtensionSetpoint(double extensionSetpoint) {
    io.setExtensionSetpoint(extensionSetpoint);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }
}
