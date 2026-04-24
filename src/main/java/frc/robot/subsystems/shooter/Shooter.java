package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase{
    
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert shooterLeftTopDisconnectedAlert;
  private final Alert shooterLeftBottomDisconnectedAlert;
  private final Alert shooterRightTopDisconnectedAlert;
  private final Alert shooterRightBottomDisconnectedAlert;
  private final Alert shooterHoodDisconnectedAlert;
  private static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shooterTimeMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap passingMap = new InterpolatingDoubleTreeMap();
  public Shooter(ShooterIO io) {
    this.io = io;
    shooterLeftTopDisconnectedAlert = new Alert("Left Top Flywheel Disconnected", AlertType.kError);
    shooterLeftBottomDisconnectedAlert = new Alert("Left Bottom Flywheel Disconnected", AlertType.kError);
    shooterRightTopDisconnectedAlert = new Alert("Right Top Flywheel Disconnected", AlertType.kError);
    shooterRightBottomDisconnectedAlert = new Alert("Right Bottom Flywheel Disconnected", AlertType.kError);
    shooterHoodDisconnectedAlert = new Alert("Hood Disconnected", AlertType.kError);

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
    shooterHoodDisconnectedAlert.set(!inputs.hoodConnected);
    Leds.getInstance().shooterDisconnected = 
      !inputs.flywheelLeftTopConnected || 
      !inputs.flywheelRightTopConnected || 
      !inputs.flywheelLeftBottomConnected || 
      !inputs.flywheelRightBottomConnected || 
      !inputs.hoodConnected;
    // This method will be called once per scheduler run
  }

  private void setHoodAngleMap(){

    //CLOSE RANGE SPEED
    hoodAngleMap.put(1.397, 2.0);
    hoodAngleMap.put(1.8034, 4.0);
    hoodAngleMap.put(2.2098, 6.0);
    hoodAngleMap.put(2.667, 8.0);
    hoodAngleMap.put(2.9, 8.5);

    //FAR RANGE SPEED
    hoodAngleMap.put(3.0,7.25);
    hoodAngleMap.put(3.4, 7.75);
    hoodAngleMap.put(3.429, 8.5);
    hoodAngleMap.put(4.0, 9.4);
    hoodAngleMap.put(4.5, 10.7);

    //SUPER FAR RANGE
    hoodAngleMap.put(5.0,9.5);
    hoodAngleMap.put(5.5, 11.0);

    hoodAngleMap.put(7.1, 11.0);
    hoodAngleMap.put(11.0, 13.1);

    //TIME OF FLIGHT
    shooterTimeMap.put(1.19, 1.25);
    shooterTimeMap.put(1.7, 1.3);
    shooterTimeMap.put(2.2, 1.32);
    shooterTimeMap.put(2.9, 1.22);
    shooterTimeMap.put(3.2, 1.55);
    shooterTimeMap.put(3.8, 1.31);
    shooterTimeMap.put(4.1, 1.3);
    shooterTimeMap.put(4.5, 1.33);
    shooterTimeMap.put(4.9, 1.43);
    shooterTimeMap.put(5.5, 1.41);

    passingMap.put(6.0, 65.0);
    passingMap.put(8.0, 70.0);
    passingMap.put(9.6, 78.0);
    passingMap.put(10.4, 85.0);
    passingMap.put(12.0, 95.0);

  }

  public double interpolateHoodAngle(double distanceMeters){
      return hoodAngleMap.get(distanceMeters);
  }

  public static double interpolateShooterTime(double distanceMeters){
      return shooterTimeMap.get(distanceMeters);
  }

  public double getFlywheelShootSpeed(double distanceMeters){
    if(distanceMeters > 7){
      return 76.0;
    }
    if(distanceMeters > 4.8){
      return 64.0;
    }
    if(distanceMeters > 3.048){
      return 58.0;
    }
    return 52.0;
  }

  public double interpolatePassSpeed(double distanceMeters){
    return passingMap.get(distanceMeters);
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
    return Math.abs(inputs.flywheelLeftTopVelocity - rotationsPerSecond) < 3.0;
  }
  @AutoLogOutput(key = "Shooter/rightFlywheelUpToSpeed")
  public boolean rightFlywheelUpToSpeed(double rotationsPerSecond){
    return Math.abs(inputs.flywheelRightTopVelocity - rotationsPerSecond) < 3.0;
  }

  public Command lerpHood(DoubleSupplier distance){
      return Commands.run(() -> this.setHoodAngle(interpolateHoodAngle(distance.getAsDouble())), this);
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
