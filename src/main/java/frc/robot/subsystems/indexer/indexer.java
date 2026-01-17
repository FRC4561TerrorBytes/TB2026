// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.indexerIO.indexerIOInputs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {
  
  private indexerIO io;
  private indexerIOInputsAutoLogged inputs = new indexerIOInputsAutoLogged();
  private final Alert indexerDisconnectedAlert;

  public indexer(indexerIO io){
    this.io = io;
    indexerDisconnectedAlert = new Alert("Indexer Freaking Disconnected Bruh", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("indexer/IO", inputs);
    indexerDisconnectedAlert.set(!inputs.indexerConnected);
    // This method will be called once per scheduler run
  }

  public void setOutput(double speed){
    io.setOutput(speed);
  }
}
