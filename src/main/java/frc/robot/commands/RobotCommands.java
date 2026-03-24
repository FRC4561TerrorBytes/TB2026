package frc.robot.commands;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotCommands {
    public static Command shoot(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Indexer indexer, Intake intake, Extension extension, Shooter shooter){
        return Commands.parallel(
            new AutoShootCommand(drive, xSupplier, ySupplier, indexer, shooter),
            agitateBalls(intake, extension)
        );
    }

    public static Command agitateBalls(Intake intake, Extension extension){
        return Commands.repeatingSequence(Commands.parallel(
                Commands.sequence(
                        Commands.runOnce(()-> intake.setOutput(0.6), intake),
                        Commands.waitSeconds(1),
                        Commands.runOnce(()-> intake.setOutput(-0.2), intake),
                        Commands.waitSeconds(0.3)),
                Commands.sequence(
                        Commands.runOnce(()-> extension.setExtensionSetpoint(0.175), extension),
                        Commands.waitSeconds(0.6),
                        Commands.runOnce(()-> extension.setExtensionOutput(Constants.EXTENSION_EXTENDED_POSITION), extension),
                        Commands.waitSeconds(0.6))
                ));
    }

    public static Command autoClimb(Drive drive, Extension extension, Climber climber){
        return Commands.sequence(
            Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION), extension), 
            climber.climbUp(), 
            drive.driveToClimbPose(2.5,1,40,20,0), 
            drive.driveUntilObstruction(new ChassisSpeeds(-0.5,0,0), 3), 
            climber.climbDown()
            );
    }

    public static Command driverRumbleCommand(CommandXboxController driverController) {
        return Commands.startEnd(
                () -> {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        Logger.recordOutput("RobotContainer/Rumbling", true);
                },
                () -> {
                        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        Logger.recordOutput("RobotContainer/Rumbling", false);
                });
    }
}
