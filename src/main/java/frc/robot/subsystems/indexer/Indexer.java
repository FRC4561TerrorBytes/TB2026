// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
//HI  MANBIR -SARAH 
//hey give me my computer back
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private IndexerIO io;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Alert indexerRightDisconnectedAlert;
  private final Alert indexerLeftDisconnectedAlert;
  private final Alert fuelKickerDisconnectedAlert;

  public Indexer(IndexerIO io) {
    this.io = io;
    indexerRightDisconnectedAlert =
        new Alert("Indexer Right Disconnected", AlertType.kError);
    indexerLeftDisconnectedAlert =
        new Alert("Indexer Left Disconnected", AlertType.kError);
    fuelKickerDisconnectedAlert =
        new Alert("Fuel Kicker Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/IO", inputs);
    indexerRightDisconnectedAlert.set(!inputs.indexerRightConnected);
    indexerLeftDisconnectedAlert.set(!inputs.indexerLeftConnected);
    fuelKickerDisconnectedAlert.set(!inputs.fuelKickerConnected);
    // This method will be called once per scheduler run
  }

  public void setIndexerVoltage(double voltage) {
    io.setLeftIndexerVoltage(voltage);
    io.setRightIndexerVoltage(voltage);
  }

  public void setLeftIndexerVoltage(double voltage){
    io.setLeftIndexerVoltage(voltage);
  }

  public void setRightIndexerVoltage(double voltage){
    io.setRightIndexerVoltage(voltage);
  }

  public void setFuelKickerVoltage(double voltage) {
    io.setFuelKickerVoltage(voltage);
  }

  public void setThroughput(double indexerVoltage, double kickerVoltage) {
    setFuelKickerVoltage(kickerVoltage);
    setIndexerVoltage(indexerVoltage);
  }

  public void stop(){
    this.setThroughput(0.0, 0.0);
  }
}