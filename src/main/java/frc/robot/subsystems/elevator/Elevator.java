// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert ElevatorDisconnectedAlert;
  /** Creates a new Extension. */
  public Elevator(ElevatorIO io) {
    this.io = io;
    ElevatorDisconnectedAlert = new Alert("Elevator motor disconnected", AlertType.kError);
  }
    public void setElevatorSetpoint(double elevatorSetpoint) {
      io.setElevatorSetpoint(elevatorSetpoint);
  }

  public double elevatorSetpoint() {
    return inputs.elevatorSetpoint;
  }

  @Override
  public void periodic() {
    
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/Io", inputs);
    ElevatorDisconnectedAlert.set(!inputs.elevatorMotorConnected);
    // This method will be called once per scheduler run
  }
}
