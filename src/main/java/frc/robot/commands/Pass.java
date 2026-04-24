// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class Pass extends Command {

  public Drive drive;
  public Indexer indexer;
  public Shooter shooter;
  
  /**
   * Shoots forward pretty much, auto makes it aim at bump and shoot.
   */
  public Pass(Drive drive, Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(indexer, shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    double flywheelRPS = shooter.interpolatePassSpeed(drive.getDistanceToPassPoint());
    //double hoodAngle = shooter.interpolateHoodAngle(drive.getDistanceToPassPoint());
    double hoodAngle = 14.1;

    shooter.setFlywheelSpeed(flywheelRPS);
    shooter.setHoodAngle(hoodAngle);

      if(shooter.leftFlywheelUpToSpeed(flywheelRPS) && shooter.rightFlywheelUpToSpeed(flywheelRPS) && shooter.hoodAtSetpoint()){
        indexer.setThroughput(0.9, 0.8);
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
