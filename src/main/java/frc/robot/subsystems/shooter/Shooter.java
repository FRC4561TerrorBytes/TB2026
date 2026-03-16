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
  private static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap shooterTimeMap = new InterpolatingDoubleTreeMap();
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

    hoodAngleMap.put(Units.inchesToMeters(62), 0.0);
    //shooterTimeMap.put(Units.inchesToMeters(62),1.0);

    hoodAngleMap.put(Units.inchesToMeters(55), 2.0);
    hoodAngleMap.put(Units.inchesToMeters(71), 4.0);
    hoodAngleMap.put(Units.inchesToMeters(87), 6.0);
    hoodAngleMap.put(Units.inchesToMeters(105), 8.0);

    hoodAngleMap.put(Units.inchesToMeters(135), 8.5);
    hoodAngleMap.put(3.0,7.25);
    hoodAngleMap.put(3.4, 7.75);
    hoodAngleMap.put(4.0, 9.4);
    hoodAngleMap.put(5.0,11.0);

    shooterTimeMap.put(1.38, 0.45);
    shooterTimeMap.put(1.88, 0.55);
    shooterTimeMap.put(3.15, 0.60);
    shooterTimeMap.put(4.55, 0.65);
    shooterTimeMap.put(5.68, 0.70);
  }

  public double interpolateHoodAngle(double distanceMeters){
      return hoodAngleMap.get(distanceMeters);
  }

  public static double interpolateShooterTime(double distanceMeters){
      return shooterTimeMap.get(distanceMeters);
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
