// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class snap45 extends Command {
  /** Creates a new AutoRotate. */

  final Drive drive;
  final DoubleSupplier xSupplier;
  final DoubleSupplier ySupplier;
  double startAngle;
  double degreesClosestTo;
  double angle;
  private final Alert snapToDiagonal =
      new Alert("it isnt goin to 45° :( do it manually now ", AlertType.kError);
      
   public snap45(
    Drive drive,
    DoubleSupplier xSupplier, 
    DoubleSupplier ySupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Target Angle", degreesClosestTo - startAngle);
    System.out.println(drive.getPose().getRotation().getDegrees() + 180);
    System.out.println("rotation from pose: " + (drive.getPose().getRotation().getDegrees() + 180));
      DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> Rotation2d.fromDegrees((degreesClosestTo - startAngle) % 360)); 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(drive.getPose().getRotation().getDegrees() - ((degreesClosestTo - startAngle) % 360))) < 1.0);
  }
}