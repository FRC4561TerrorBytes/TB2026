// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveUntilObstruction extends Command {
  Drive drive;
  double time;
  /** Creates a new DriveUntilObstruction. */
  public DriveUntilObstruction(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = System.currentTimeMillis();
  }

  ChassisSpeeds speeds = new ChassisSpeeds(2, 0, 0);
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (drive.getModulesAvgDriveCurrent() > TunerConstants.kSlipCurrent.magnitude() && drive.getChassisSpeeds().vxMetersPerSecond <= 0.2) || time + Units.secondsToMilliseconds(6) >= System.currentTimeMillis();
  }
}
