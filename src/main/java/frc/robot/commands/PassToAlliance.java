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

public class PassToAlliance extends Command {

  public Drive drive;
  public Indexer indexer;
  public Shooter shooter;
  public boolean autoAim;

  public double shootSpeedRPS = 40;
  public double hoodAngle = 6;
  
  /**
   * Shoots forward pretty much, auto makes it aim at bump and shoot.
   */
  public PassToAlliance(Drive drive, Indexer indexer, Shooter shooter) {
    this.drive = drive;
    this.indexer = indexer;
    this.shooter = shooter;
    this.autoAim = autoAim;
    addRequirements(drive, indexer, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

      shooter.setHoodAngle(hoodAngle);

      Logger.recordOutput("LeftFlywheelUpToSpeed", shooter.leftFlywheelUpToSpeed(shootSpeedRPS));
      Logger.recordOutput("RightFlywheelUpToSpeed", shooter.rightFlywheelUpToSpeed(shootSpeedRPS));

      if(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) &&shooter.hoodAtSetpoint()){
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
