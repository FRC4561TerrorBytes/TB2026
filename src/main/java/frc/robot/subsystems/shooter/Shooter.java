package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase{
    
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert shooterLeftDisconnectedAlert;
  private final Alert shooterRightDisconnectedAlert;

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterLeftDisconnectedAlert = new Alert("Left Flywheel Freaking Disconnected Bruh", AlertType.kError);
    shooterRightDisconnectedAlert = new Alert("Right Flywheel Freaking Disconnected Bruh", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/IO", inputs);
    shooterLeftDisconnectedAlert.set(!inputs.flywheelLeftConnected);
    shooterRightDisconnectedAlert.set(!inputs.flywheelRightConnected);
    // This method will be called once per scheduler run
  }

  public void setFlywheelsVoltage(double speed) {
    io.setFlywheelLeftVoltage(speed);
    io.setFlywheelRightVoltage(speed);
  }

  public Command shoot(){
    return Commands.run(() -> this.setFlywheelsVoltage(1), this);
  }

  public Command stop(){
    return Commands.run(() -> this.setFlywheelsVoltage(0), this);
  }
}
