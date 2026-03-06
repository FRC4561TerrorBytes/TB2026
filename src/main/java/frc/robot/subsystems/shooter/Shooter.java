package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  private InterpolatingDoubleTreeMap hoodAngleMapClose = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap hoodAngleMapFar = new InterpolatingDoubleTreeMap();

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

    //hoodAngleMapClose.put(Units.inchesToMeters(135), 6.0);

    hoodAngleMapClose.put(Units.inchesToMeters(62), 0.0);
    hoodAngleMapClose.put(Units.inchesToMeters(55), 2.0);
    hoodAngleMapClose.put(Units.inchesToMeters(71), 4.0);
    hoodAngleMapClose.put(Units.inchesToMeters(87), 6.0);
    hoodAngleMapClose.put(Units.inchesToMeters(105), 8.0);

    hoodAngleMapFar.put(Units.inchesToMeters(135), 8.5);

    hoodAngleMapFar.put(Units.inchesToMeters(200), 8.0);
    hoodAngleMapFar.put(Units.inchesToMeters(240), 12.0);
  }

  public double interpolateHoodAngle(double distanceMeters){
    //2 IS ARITRARY DISTANCE
    if(distanceMeters > Units.inchesToMeters(105)){
      return hoodAngleMapFar.get(distanceMeters);
    }
    else{
      return hoodAngleMapClose.get(distanceMeters);
    }
  }

  public double getFlywheelShootSpeed(double distanceMeters){
    if(distanceMeters > Units.inchesToMeters(120)){
      return 58.0;
    }
    else{
      return 52.0;
    }
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
    return Math.abs(inputs.flywheelLeftTopVelocity - rotationsPerSecond) < 6.0;
  }
  @AutoLogOutput(key = "Shooter/rightFlywheelUpToSpeed")
  public boolean rightFlywheelUpToSpeed(double rotationsPerSecond){
    return Math.abs(inputs.flywheelRightTopVelocity - rotationsPerSecond) < 6.0;
  }

  public void idleFlywheels(){
    io.setLeftFlywheelSpeed(20);
    io.setRightFlywheelSpeed(20);
  }
  

  public void stop(){
    setFlywheelVoltage(0);
  }

  public Pose3d getHoodPose(){
    return new Pose3d(-0.084,0,0.39, new Rotation3d(0,(182*Math.PI)/180,0));
  }
}
