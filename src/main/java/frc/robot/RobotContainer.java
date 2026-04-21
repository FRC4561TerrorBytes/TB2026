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

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoShootCommand; 
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Pass;
import frc.robot.commands.RobotCommands;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.extension.ExtensionIOReal;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
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
    

    Rotation2d snapRotation;
    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController testingController = new CommandXboxController(2);

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
                intake = new Intake(new IntakeIOSim()); // for actual code use intake IO sim
                extension = new Extension(new ExtensionIOSim());
                shooter = new Shooter(
                        new ShooterIOSim());
                indexer = new Indexer(new IndexerIOSim());
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
                break;
        }

        // Register NamedCommands for use in PathPlanner
        // Set up auto routines
        NamedCommands.registerCommand("intake", Commands.run(() -> intake.setOutput(Constants.INTAKE_SPEED), intake));
        NamedCommands.registerCommand("intakefast", Commands.run(() -> intake.setOutput(Constants.INTAKE_SPEED), intake));
        NamedCommands.registerCommand("stopintake", Commands.runOnce(() -> intake.setOutput(0), intake));
        NamedCommands.registerCommand("shoot", RobotCommands.shootWithAgitate(drive, intake, extension, indexer, shooter).withTimeout(5.0));
        NamedCommands.registerCommand("shootpreload", RobotCommands.shootPreload(drive, indexer, shooter).withTimeout(2.5));
        NamedCommands.registerCommand("hoodup", Commands.runOnce(() -> shooter.setHoodAngle(6), shooter));
        NamedCommands.registerCommand("slapdown", Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION)));
        NamedCommands.registerCommand("retractintake", Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION)));
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
        shooter.setDefaultCommand(Commands.runOnce(() -> shooter.stop(), shooter).andThen(shooter.lerpHood(drive::getDistanceToHub)));

        //TRIGGERS
        new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 30)
        .onTrue(
            RobotCommands.driverRumbleCommand(driverController)
                .withTimeout(2.0)
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
                
        // DRIVER CONTROLS
        driverController
                .leftTrigger()
                .toggleOnTrue(Commands.sequence(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION), extension),
                        Commands.waitUntil(() -> extension.atSetPoint(0.15)),
                        Commands.run(() -> intake.setOutput(Constants.INTAKE_SPEED), intake)));

        driverController
                .b() // retract intake
                .onTrue(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION),
                                extension).andThen(Commands.sequence(Commands.runOnce(() -> intake.setOutput(Constants.INTAKE_SPEED), intake), Commands.waitSeconds(0.5), Commands.runOnce(() -> intake.setOutput(0.0), intake))));
        driverController
                .x()
                .whileTrue(Commands.run(() -> drive.stopWithX()));

        driverController
                .rightTrigger()
                .whileTrue(RobotCommands.shoot(drive, driverController::getLeftX, driverController::getLeftY, indexer, shooter))
                .whileTrue(Commands.run(() -> Leds.getInstance().autoScoring = true))
                .onFalse(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION), extension))
                .onFalse(Commands.runOnce(() -> Leds.getInstance().autoScoring = false));

        driverController.rightBumper().whileTrue(new Pass(drive, indexer, shooter)).onTrue(Commands.runOnce(() -> Leds.getInstance().passing = true)).onFalse(Commands.runOnce(() -> Leds.getInstance().passing = false));

        driverController
                .rightStick()
                .and(driverController.leftStick())
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                                .ignoringDisable(true));

        //driverController.y().whileTrue(Commands.run(() -> indexer.setThroughput(-0.4, -0.4)));
        driverController.povDown().toggleOnTrue(Commands.run(() -> shooter.setHoodAngle(0)));

        driverController.y().whileTrue(RobotCommands.agitateBalls(intake, extension));

        driverController.povUp().whileTrue(Commands.run(() -> intake.setOutput(-Constants.INTAKE_SPEED), intake));

        //OPERATOR CONTROLS
        operatorController
                .y()
                .whileTrue(new Shoot(indexer, shooter, 20, 10));

        operatorController.leftTrigger() // extend and run intake
                .onTrue(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION), extension))
                .toggleOnTrue(
                        Commands.startRun(
                                () -> Leds.getInstance().intakeRunning = true, 
                                () -> intake.setOutput(Constants.INTAKE_SPEED), 
                                intake)
                        .alongWith(RobotCommands.driverRumbleCommand(driverController))
                        .finallyDo(() -> Leds.getInstance().intakeRunning = false));
        
        operatorController
                .rightTrigger()
                .whileTrue(new AutoShootCommand(drive, driverController::getLeftX, driverController::getLeftX, indexer, shooter));

        operatorController
                .b() // retract intake
                .onTrue(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION),
                                extension).andThen(Commands.sequence(Commands.runOnce(() -> intake.setOutput(Constants.INTAKE_SPEED), intake), Commands.waitSeconds(0.5), Commands.runOnce(() -> intake.setOutput(0.0), intake))));

        testingController.leftTrigger().onTrue(
                        Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_EXTENDED_POSITION), extension));
        testingController.b().onTrue(Commands.runOnce(() -> extension.setExtensionSetpoint(Constants.EXTENSION_RETRACTED_POSITION), extension));
        testingController.rightTrigger().whileTrue(Commands.run(() -> shooter.setFlywheelSpeed(62)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void autoExit() {
       
    }

    public void teleopEnter(){
        
    }
}
