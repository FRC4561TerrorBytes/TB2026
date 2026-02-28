// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {

  private Drive drive;
  private int endTagId;
  private boolean seenEndTag;
  private Command pathCommand;

  private Supplier <Pose2d> targetPose;
  private Pose2d currentTarget;
  private double tolerance;
  private boolean scoreBack = true;

  ProfiledPIDController xController =
      new ProfiledPIDController(15, 0, 0, new TrapezoidProfile.Constraints(20, 20));
  ProfiledPIDController yController =
      new ProfiledPIDController(15, 0, 0, new TrapezoidProfile.Constraints(20, 20));
  ProfiledPIDController thetaController =
      new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

  /** Creates a new DriveToPose. */
  public DriveToPose(Drive drive, Supplier <Pose2d> targetPose, double tolerance) {
    this.drive = drive;
    this.targetPose = targetPose;
    this.tolerance = tolerance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    currentTarget = targetPose.get();
    currentTarget = AllianceFlipUtil.apply(currentTarget);
    Logger.recordOutput("Auto Lineup/Target Pose", currentTarget);

    if (drive.getPose().getTranslation().getDistance(currentTarget.getTranslation()) > 1) {
      pathCommand =
          AutoBuilder.pathfindToPose(currentTarget, new PathConstraints(3, 2, Math.PI, Math.PI), 0);

      pathCommand.withName("DriveToPose").schedule();
    } else {
      pathCommand = null;

      xController.setGoal(currentTarget.getX());
      xController.setTolerance(tolerance);
      xController.reset(drive.getPose().getX());
      yController.setGoal(currentTarget.getY());
      yController.setTolerance(tolerance);
      yController.reset(drive.getPose().getY());
      thetaController.setGoal(currentTarget.getRotation().getRadians());
      thetaController.setTolerance(Units.degreesToRadians(0.2));
      thetaController.reset(drive.getPose().getRotation().getRadians());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pathCommand == null) {
      double xSpeed = xController.calculate(drive.getPose().getX());
      double ySpeed = yController.calculate(drive.getPose().getY());
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      double rotSpeed = thetaController.calculate(drive.getPose().getRotation().getRadians());

      Logger.recordOutput("Auto Lineup/RunXVelocity", xSpeed);
      Logger.recordOutput("Auto Lineup/TargetPoseX", currentTarget.getX());
      Logger.recordOutput("Auto Lineup/RunYVelocity", ySpeed);
      Logger.recordOutput("Auto Lineup/TargetPoseY", currentTarget.getY());
      Logger.recordOutput("Auto Lineup/RunThetaVelocity", rotSpeed);
      // Hi Ethan :)

      // feed to chassis
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, rotSpeed, drive.getPose().getRotation());
      drive.runVelocity(speeds);
    }
    Logger.recordOutput("TEST/score back", scoreBack);
    Logger.recordOutput(
        "TEST/target dist",
        drive.getPose().getTranslation().getDistance(currentTarget.getTranslation()));
    Logger.recordOutput("Auto Lineup/Seen Tag", seenEndTag);
    Logger.recordOutput("Auto Lineup/Tag ID", endTagId);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      // stopping the drive since the path was cut short and therefore robot still moving
      // else the path has been completed and stopped by the path since end velocity is 0
      drive.stop();
    }
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathCommand != null) {
      switch (Constants.currentMode) {
        case REAL:
          return pathCommand.isFinished();
        case SIM:
          return pathCommand.isFinished();
        case REPLAY:
          return true;
        default:
          return true;
      }
    } else {
      return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
  }
}