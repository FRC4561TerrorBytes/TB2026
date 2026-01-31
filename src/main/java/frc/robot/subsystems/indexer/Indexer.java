package frc.robot.subsystems.indexer;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private IndexerIO io;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Alert indexerDisconnectedAlert;

  public Indexer(IndexerIO io) {
    this.io = io;
    indexerDisconnectedAlert = new Alert("Indexer Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IO", inputs);
    indexerDisconnectedAlert.set(!inputs.indexerConnected);
    // This method will be called once per scheduler run
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public Command spin(){
    return Commands.run(() -> this.setOutput(1), this);
  }

  public Command stop(){
    return Commands.run(() -> this.setOutput(0), this);
  }
}