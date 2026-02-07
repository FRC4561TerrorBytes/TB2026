// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class snap45 extends Command {
  /** Creates a new AutoRotate. */

  final Drive drive;
  final PIDController m_pidController = new PIDController(0.025, 0.01, 0);
  double startAngle;
  double degreesClosestTo;
  double angle;
  private final Alert snapToDiagonal =
      new Alert("it isnt goin to 45° :( do it manually now ", AlertType.kError); //hi manbir

  public snap45(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    m_pidController.enableContinuousInput(0, 360);
    //m_pidController.setSetpoint(0.0);
    m_pidController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.reset();
    //final double absAngle = Math.abs(drive.getPose().getRotation().getDegrees());
    double degreesClosestTo = 0;
    startAngle = Units.radiansToDegrees(Math.atan(25.0/30.0));
    angle = drive.getPose().getRotation().getDegrees() + 180;
    double correctedAngle = angle - startAngle;
    if ( -startAngle < correctedAngle && correctedAngle < (90 - startAngle)){
        degreesClosestTo = 90;
    }
    else if ((90 - startAngle) < correctedAngle && correctedAngle < (180 - startAngle)){
        degreesClosestTo = 180;
    }
    else if ((180 - startAngle) < correctedAngle && correctedAngle < (270 - startAngle)){
      degreesClosestTo = 270;
    }
    else if ((270 - startAngle) < correctedAngle && correctedAngle < (-startAngle)){
      degreesClosestTo = 0;
    }
    else {
      degreesClosestTo = 0;
      snapToDiagonal.set(true);
    }
    
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("rotating to", (degreesClosestTo - startAngle));
    m_pidController.setSetpoint(degreesClosestTo - startAngle);

    //final boolean closerTo0 = (180.0 - absAngle) > absAngle;
    //m_pidController.setSetpoint(closerTo0 ? 0.0 : 180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Target Angle", degreesClosestTo - startAngle);
    double rotationRate = m_pidController.calculate(angle);  
    System.out.println(drive.getPose().getRotation().getDegrees() + 180);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0.0, 0.0, rotationRate), drive.getPose().getRotation()));

    System.out.println("rotation from pose: " + (drive.getPose().getRotation().getDegrees() + 180));
    // System.out.println("rotation from pigeon: " + drive.getPigeonYaw());

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