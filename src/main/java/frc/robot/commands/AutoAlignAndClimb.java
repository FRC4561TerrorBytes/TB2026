// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignAndClimb extends Command {

  Drive drive;
  Climber climb;
  boolean rightCloser;
  Pose2d pose;
  Command path;


  private Pose2d rightClimb;
  private Pose2d leftClimb;
  Pose2d targetPose;
  public AutoAlignAndClimb(Drive drive, Climber climb) {
    this.drive = drive;
    this.climb = climb;

    addRequirements(drive,climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightClimb = AllianceFlipUtil.apply(new Pose2d(1.442,3.416714, new Rotation2d()));
    leftClimb = AllianceFlipUtil.apply(new Pose2d(1.442,4.03101, new Rotation2d()));

    pose = drive.getPose();
    if(rightClimb.getTranslation().getDistance(pose.getTranslation()) < leftClimb.getTranslation().getDistance(pose.getTranslation()))
      targetPose = rightClimb;
    else
      targetPose = leftClimb;

    path = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(2, 1, 180, 180));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.climbUp();

    path.schedule();
    if(path.isFinished()){
      climb.climbDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    path.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
