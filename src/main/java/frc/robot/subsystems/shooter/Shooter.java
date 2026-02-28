package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
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
  private final Alert shooterLeftTopDisconnectedAlert;
  private final Alert shooterLeftBottomDisconnectedAlert;
  private final Alert shooterRightTopDisconnectedAlert;
  private final Alert shooterRightBottomDisconnectedAlert;
  private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterLeftTopDisconnectedAlert = new Alert("Left Top Flywheel Disconnected", AlertType.kError);
    shooterLeftBottomDisconnectedAlert = new Alert("Left Bottom Flywheel Disconnected", AlertType.kError);
    shooterRightTopDisconnectedAlert = new Alert("Right Top Flywheel Disconnected", AlertType.kError);
    shooterRightBottomDisconnectedAlert = new Alert("Right Bottom Flywheel Disconnected", AlertType.kError);

    setHoodAngleMap();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/IO", inputs);
    shooterLeftTopDisconnectedAlert.set(!inputs.flywheelLeftTopConnected);
    shooterLeftBottomDisconnectedAlert.set(!inputs.flywheelLeftBottomConnected);
    shooterRightTopDisconnectedAlert.set(!inputs.flywheelRightTopConnected);
    shooterRightBottomDisconnectedAlert.set(!inputs.flywheelRightBottomConnected);
    // This method will be called once per scheduler run
  }

  private void setHoodAngleMap(){
    //hoodAngleMap.put(Units.inchesToMeters(67), 41.0);
    //ideally we have a whole ton more entries here but we lowk need robot for that 🙃
    hoodAngleMap.put(Units.inchesToMeters(240), 9.8);

    //HI MIKEY
  }

  public double interpolateHoodAngle(double distanceMeters){
    return hoodAngleMap.get(distanceMeters);
  }

  public void setHoodAngle(double angle){
    io.setHoodAngle(angle);
  }

  public double getHoodAngle(){
    return inputs.hoodRelativePosition;
  }

  @AutoLogOutput
  public boolean hoodAtSetpoint(){
    return Math.abs(inputs.hoodRelativePosition - inputs.hoodSetpoint) < 0.6;
  }

  public void nudge(double amount){
      setHoodAngle(amount + getHoodAngle());
  }

  public void setFlywheelVoltage(double voltage) {
    io.setLeftFlywheelVoltage(voltage);
    io.setRightFlywheelVoltage(voltage);
  }

  public void setFlywheelSpeed(double velocityRPS){
    io.setLeftFlywheelSpeed(velocityRPS);
    io.setRightFlywheelSpeed(velocityRPS);
  }

  @AutoLogOutput(key = "Shooter/leftFlywheelUpToSpeed")
  public boolean leftFlywheelUpToSpeed(double rotationsPerSecond){
    return inputs.flywheelLeftTopVelocity >= (rotationsPerSecond*0.95);
  }
  @AutoLogOutput(key = "Shooter/rightFlywheelUpToSpeed")
  public boolean rightFlywheelUpToSpeed(double rotationsPerSecond){
    return inputs.flywheelRightTopVelocity >= (rotationsPerSecond*0.95);
  }

  public void idleFlywheels(){
    io.setLeftFlywheelSpeed(20);
    io.setRightFlywheelSpeed(20);
  }
  

  public void stop(){
    setFlywheelVoltage(0);
  }
}
