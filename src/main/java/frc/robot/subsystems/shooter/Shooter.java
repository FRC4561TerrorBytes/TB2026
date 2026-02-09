package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase{
    
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert shooterDisconnectedAlert;
  private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterDisconnectedAlert = new Alert("Shooter Freaking Disconnected Bruh", AlertType.kError);

    setHoodAngleMap();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/IO", inputs);
    shooterDisconnectedAlert.set(!inputs.flywheelsConnected);
    // This method will be called once per scheduler run
  }

  private void setHoodAngleMap(){
    //hoodAngleMap.put(Units.inchesToMeters(67), 41.0);
    //ideally we have a whole ton more entries here but we lowk need robot for that 🙃
  }

  public double interpolateHoodAngle(double distanceMeters){
    return hoodAngleMap.get(distanceMeters);
  }

  public void setFlywheelsVoltage(double speed) {
    io.setFlywheelsVoltage(speed);
  }

  public Command shoot(){
    return Commands.run(() -> this.setFlywheelsVoltage(1), this);
  }

  public Command stop(){
    return Commands.run(() -> this.setFlywheelsVoltage(0), this);
  }
}
