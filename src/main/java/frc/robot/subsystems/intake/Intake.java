// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert IntakeLeftDisconnectedAlert;
  private final Alert IntakeRightDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    IntakeLeftDisconnectedAlert = new Alert("Left intake motor disconnected", AlertType.kError);
    IntakeRightDisconnectedAlert = new Alert("Right intake motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Io", inputs);
    IntakeLeftDisconnectedAlert.set(!inputs.intakeLeftMotorConnected);
    IntakeRightDisconnectedAlert.set(!inputs.intakeRightMotorConnected);
    Leds.getInstance().intakeDisconncted = !inputs.intakeLeftMotorConnected || !inputs.intakeRightMotorConnected;
  }
  
  
  public void setOutput(double speed) {
    io.setOutput(speed);
  }
}
