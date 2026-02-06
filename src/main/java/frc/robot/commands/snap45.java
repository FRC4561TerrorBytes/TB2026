// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class snap45 extends Command {
  /** Creates a new AutoRotate. */

  final Drive drive;
  final PIDController m_pidController = new PIDController(0.025, 0.01, 0);

  public snap45(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    m_pidController.enableContinuousInput(-180.0, 180.0);
    //m_pidController.setSetpoint(0.0);
    m_pidController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();
    //final double absAngle = Math.abs(drive.getPose().getRotation().getDegrees());
    final double angle = drive.getPose().getRotation().getDegrees() + 180;
    double degreesClosestTo = 0;
    double closest = 999.0;
    double targetAngle = Units.radiansToDegrees(Math.atan(25.0/30.0));
    
    
    if (Math.abs(angle - (targetAngle - 360)) < closest){
      closest = Math.abs(angle - (targetAngle - 360));
      degreesClosestTo = (targetAngle - 360);
    }
    if (Math.abs(angle - (targetAngle - 270)) < closest){
      closest = Math.abs(angle - (targetAngle)) - 270;
      degreesClosestTo = (targetAngle) - 270;
    }
    if (Math.abs(angle - (targetAngle - 180)) < closest){
      closest = Math.abs(angle - (targetAngle - 180));
      degreesClosestTo = targetAngle - 180;
    }
    if (Math.abs(angle - (targetAngle - 90)) < closest){
      closest = Math.abs(angle - (targetAngle - 90));
      degreesClosestTo = targetAngle - 90;
    }
    if(Math.abs(angle - targetAngle) < closest){
      closest = Math.abs(angle);
      degreesClosestTo = 0;
    }
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("rotating to", degreesClosestTo);
    m_pidController.setSetpoint(degreesClosestTo);

    //final boolean closerTo0 = (180.0 - absAngle) > absAngle;
    //m_pidController.setSetpoint(closerTo0 ? 0.0 : 180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawAngle = drive.getPose().getRotation().getDegrees();
    double rotationRate = m_pidController.calculate(rawAngle + 180);    
    rotationRate += 1.2 * Math.signum(rotationRate);
    System.out.println(drive.getPose().getRotation().getDegrees() + 180);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, rotationRate), drive.getPose().getRotation()));

    System.out.println("rotation from pose: " + (drive.getPose().getRotation().getDegrees() + 180));
    // System.out.println("rotation from pigeon: " + drive.getPigeonYaw());

    SmartDashboard.putNumber("Raw Angle", rawAngle);
    SmartDashboard.putNumber("Rotation Rate", rotationRate);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}