// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoShootAndMoveCommand;
import frc.robot.commands.BetterAutoShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterSpeedup;
import frc.robot.commands.ShotAlignAndStop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim; 
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.extension.ExtensionIOReal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.RobotVisualizer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Extension extension;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Leds leds;
  private final Climber climber;

    Rotation2d snapRotation;
    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                intake = new Intake(new IntakeIOReal());
                extension = new Extension(new ExtensionIOReal());
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(camera0Name, drive::getRotation),
                        new VisionIOLimelight(camera1Name, drive::getRotation));
                shooter = new Shooter(
                        new ShooterIOReal());
                indexer = new Indexer(new IndexerIOReal());
                climber = new Climber(new ClimberIOReal());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                intake = new Intake(new IntakeIO() {
                }); // for actual code use intake IO sim
                extension = new Extension(new ExtensionIO() {
                });
                shooter = new Shooter(
                        new ShooterIO() {
                        });
                indexer = new Indexer(new IndexerIO() {
                });
                climber = new Climber(new ClimberIO() {
                });
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                extension = new Extension(new ExtensionIO() {
                });
                shooter = new Shooter(
                        new ShooterIO() {
                        });
                indexer = new Indexer(new IndexerIO() {
                });
                climber = new Climber(new ClimberIO() {
                });
                break;
        }

        // Register NamedCommands for use in PathPlanner
        // Set up auto routines
        NamedCommands.registerCommand("intake", Commands.run(() -> intake.setOutput(0.8), intake));
        NamedCommands.registerCommand("stopintake", Commands.runOnce(() -> intake.setOutput(0), intake));
        NamedCommands.registerCommand("shoot", shoot().withTimeout(7.0));
         NamedCommands.registerCommand("shootpreload", new BetterAutoShootCommand(drive, indexer, shooter).withTimeout(3.5));
        NamedCommands.registerCommand("hoodup", Commands.runOnce(() -> shooter.setHoodAngle(6), shooter));
        NamedCommands.registerCommand("slapdown", Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION)));
        NamedCommands.registerCommand("retractintake", Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION)));
        NamedCommands.registerCommand("autoclimb", autoClimb());
        NamedCommands.registerCommand("climbprep", Commands.runOnce(() -> climber.setClimberPosition(Constants.CLIMBER_UP_POSITION)));
        NamedCommands.registerCommand("climbfull", Commands.runOnce(() -> climber.setClimberPosition(Constants.CLIMBER_DOWN_POSITION)));
        NamedCommands.registerCommand("spinupflywheels", Commands.run(() -> shooter.setFlywheelSpeed(40), shooter));
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // if (code no work) {code work}
        //else {make code work}
        //whoever thought of that ^^^^ is huge brain
        
        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        /*
         * autoChooser.addOption(
         * "Drive Simple FF Characterization",
         * DriveCommands.feedforwardCharacterization(drive));
         * autoChooser.addOption(
         * "Drive SysId (Quasistatic Forward)",2
         * drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
         * autoChooser.addOption(
         * "Drive SysId (Quasistatic Reverse)",
         * drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
         * autoChooser.addOption(
         * "Drive SysId (Dynamic Forward)",
         * drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
         * autoChooser.addOption(
         * "Drive SysId (Dynamic Reverse)",
         * drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
         */
        autoChooser.addOption(
                "LeaveAndStop",
                Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.25, 0, 0)), drive)
                        .withTimeout(4));

        SmartDashboard.putData(CommandScheduler.getInstance());

        RobotVisualizer.initialize(extension, shooter);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        intake.setDefaultCommand(Commands.run(() -> intake.setOutput(0), intake));
        indexer.setDefaultCommand(Commands.run(() -> indexer.stop(), indexer));
        shooter.setDefaultCommand(Commands.runOnce(() -> shooter.stop(), shooter).andThen(Commands.run(() -> shooter.setHoodAngle(shooter.interpolateHoodAngle(drive.getDistanceToHub())), shooter)));


        // DRIVER CONTROLS
        driverController
                .leftTrigger() // extend and run intake
                .onTrue(
                        Commands.sequence(Commands.runOnce(() -> climber.setClimberPosition(0.0)).beforeStarting(() -> climber.setIdleMode(NeutralModeValue.Brake)),Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION),
                                extension)))
                .toggleOnTrue(
                        Commands.run(() -> intake.setOutput(0.8), intake).alongWith(driverRumbleCommand()));

        driverController
                .b() // retract intake
                .onTrue(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION),
                                extension).andThen(Commands.sequence(Commands.runOnce(() -> intake.setOutput(0.8), intake), Commands.waitSeconds(0.5), Commands.runOnce(() -> intake.setOutput(0.0), intake))));
        driverController
                .x()
                .whileTrue(Commands.run(() -> drive.stopWithX()));

        driverController
                .rightTrigger()
                .whileTrue(shoot())
                .onFalse(
                        Commands.runOnce(()->extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION), extension));


        driverController
                .rightStick()
                .and(driverController.leftStick())
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                                .ignoringDisable(true));

        driverController.y().whileTrue(Commands.run(() -> indexer.setThroughput(-0.4, -0.4)));

        driverController.rightBumper()
                .whileTrue(climber.climbUp().beforeStarting(() -> climber.setIdleMode(NeutralModeValue.Brake)));

        driverController.leftBumper().whileTrue(climber.climbDown());

        driverController
                .povUp()
                .onTrue(Commands.runOnce(() -> shooter.nudge(0.1), shooter));

        driverController
                .povDown()
                .onTrue(Commands.runOnce(() -> shooter.nudge(-0.1), shooter));

         driverController
                .povRight()
                .whileTrue(autoClimb());

        //OPERATOR CONTROLS
        operatorController.povLeft().onTrue(climber.climbDown());
        operatorController
                .y()
                .whileTrue(new Shoot(indexer, shooter, 50, 10));

        operatorController
                .x()
                .whileTrue(Commands.parallel(
                                new AutoShootAndMoveCommand(drive, indexer, shooter),
                                DriveCommands.joystickDriveAtAngle(drive, ()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()-> drive.getRotationToHubWithVelocity())
                                ));

        operatorController.a().whileTrue(Commands.runOnce(() -> climber.setClimberPosition(0.0)).beforeStarting(() -> climber.setIdleMode(NeutralModeValue.Brake)));


        operatorController.leftTrigger().onTrue(
                Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION),
                        extension))
                .toggleOnTrue((
                        Commands.runOnce(() -> intake.setOutput(0.2), intake).alongWith(driverRumbleCommand())));
        
        operatorController
                .rightTrigger()
                .whileTrue(new BetterAutoShootCommand(drive, indexer, shooter));

        operatorController
                .b()
                .onTrue(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION),
                                extension).andThen(Commands.runOnce(() -> intake.setOutput(0.0), intake)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public Command agitateBalls(){
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

    public Command autoClimb(){
        return Commands.sequence(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION), extension), 
                        climber.climbUp(), 
                        drive.driveToClimbPose(2.5,1,40,20,0), 
                        drive.driveUntilObstruction(new ChassisSpeeds(-0.5,0,0), 3), 
                        climber.climbDown());
    }

    public Command shoot(){
        return Commands.sequence(
                new ShooterSpeedup(shooter, () -> drive.getDistanceToHub()), //initial speedup to get shooter spinning and hood up while moving
                Commands.parallel(
                        new ShotAlignAndStop(drive), 
                        new ShooterSpeedup(shooter, () -> drive.getDistanceToHub())
                        ),
                Commands.parallel(
                        Commands.runOnce(() -> indexer.setThroughput(0.6, 0.7), indexer), //hi Manbir
                        agitateBalls()
                )
        );
    }

    public void autoExit() {
        climber.setIdleMode(NeutralModeValue.Coast);
    }

    public void teleopEnter(){
        if(climber.getClimberPosition() >= Constants.CLIMBER_DOWN_POSITION){
            climber.setClimberPosition(Constants.CLIMBER_UP_POSITION);
        }
    }

    private Command driverRumbleCommand() {
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
