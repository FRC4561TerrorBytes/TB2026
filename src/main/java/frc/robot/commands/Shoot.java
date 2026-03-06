// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {

  public Drive drive;
  public Indexer indexer;
  public Shooter shooter;

  public double flyWheelRPS;
  public double hoodAngle;
  
  /**
   * Shoots forward pretty much, auto makes it aim at bump and shoot.
   */
  public Shoot(Indexer indexer, Shooter shooter, double flyWheelRPS, double hoodAngle) {
    this.hoodAngle = hoodAngle;
    this.flyWheelRPS = flyWheelRPS;
    this.drive = drive;
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(indexer, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

      shooter.setHoodAngle(hoodAngle);

      Logger.recordOutput("LeftFlywheelUpToSpeed", shooter.leftFlywheelUpToSpeed(flyWheelRPS));
      Logger.recordOutput("RightFlywheelUpToSpeed", shooter.rightFlywheelUpToSpeed(flyWheelRPS));

      if(shooter.leftFlywheelUpToSpeed(flyWheelRPS) && shooter.rightFlywheelUpToSpeed(flyWheelRPS) &&shooter.hoodAtSetpoint()){
          indexer.setThroughput(0.7, 0.6);
      }
      else{
          indexer.stop();
      }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
