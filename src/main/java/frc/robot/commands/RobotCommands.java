package frc.robot.commands;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotCommands {

    /**
     * Locks Drivetrain to orbit the hub while spinning up shooter. Once shooter is up to speed the command waits for joysticks to stop, then shoots. 
     * @return
     */
    public static Command shoot(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Indexer indexer, Shooter shooter){
        return new AutoShootCommand(drive, xSupplier, ySupplier, indexer, shooter);
    }

    /** Autoaligns the robot to the hub then locks wheels while shooting. */
    public static Command shootNoJoysticks(Drive drive, Indexer indexer, Shooter shooter){
        return new AutoShootCommand(drive, indexer, shooter);
    }

    /** Shoots the preload fuel without bringing intake in to agitate fuel. */
    public static Command shootPreload(Drive drive, Indexer indexer, Shooter shooter){
        return new AutoShootCommand(drive, indexer, shooter);
    }

    public static Command shootWithAgitate(Drive drive, Intake intake, Extension extension, Indexer indexer, Shooter shooter){
        return Commands.parallel(new AutoShootCommand(drive, indexer, shooter), agitateBalls(intake, extension));
    }

    /** Agitates the balls by moving intake in and out*/
    public static Command agitateBalls(Intake intake, Extension extension){
        return Commands.repeatingSequence(Commands.parallel(
                Commands.sequence(
                        Commands.runOnce(()-> intake.setOutput(Constants.INTAKE_SPEED), intake),
                        Commands.waitSeconds(1),
                        Commands.runOnce(()-> intake.setOutput(0.0), intake),
                        Commands.waitSeconds(0.3)),
                Commands.sequence(
                        Commands.runOnce(()-> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION), extension),
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(()-> extension.setExtensionSetpoint(Constants.EXTENSION_AGITATE_POSITION), extension),
                        Commands.waitSeconds(1.0).onlyWhile( ()-> extension.isExtentionBelowCurrent(5)))
                ));
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
