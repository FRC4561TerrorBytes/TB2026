package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class SelfTestCommands {
    private static Alert driveAlert = new Alert("Drivetrain failed bench test", AlertType.kWarning);
        private Alert climberAlert = new Alert("Climber failed bench test", AlertType.kWarning);
        private Alert extensionAlert = new Alert("Extension failed bench test", AlertType.kWarning);
        private Alert shooterAlert = new Alert("Shooter failed bench test", AlertType.kWarning);
        private Alert indexerAlert = new Alert("Indexer failed bench test", AlertType.kWarning);
        private Alert intakeAlert = new Alert("Intake failed bench test", AlertType.kWarning);
        private Alert hoodAlert = new Alert("Hood failed bench test", AlertType.kWarning);
    
     public static Command drivetrainTest(
        Drive drive) {
        return Commands.sequence(
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0)), drive).withTimeout(5),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 1, 0)), drive).withTimeout(5),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 2)), drive).withTimeout(5),
            Commands.runOnce(() -> drive.stop()),
            Commands.runOnce(
                    () -> {
                      driveAlert.set(true);
                })
            .onlyIf(
                () ->
                    drive
                        .getPose()
                        .equals(new Pose2d()))); // only if the robot doesn't move, this is true
    }


    public static Command cLimberTest(
        Climber climber) {
            return Commands.sequence(
                climber.climbUp(),
                Commands.waitSeconds(3),
                climber.climbDown(),
                Commands.waitSeconds(3)
            );
        }

    
    public static Command extensionTest(
        Extension extension) {
            return Commands.sequence(
                Commands.runOnce(() -> extension.setExtensionSetpoint(0), extension),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> extension.setExtensionSetpoint(0), extension),
                Commands.waitSeconds(3)
            );
        }

    public static Command shooterTest(
      Drive drive,
      Indexer indexer, 
      Shooter shooter) {
            return Commands.parallel(
                new AutoShootCommand(drive, indexer, shooter).withTimeout(5),
                Commands.sequence(
                    Commands.waitSeconds(2.5))
            );
        }
}
