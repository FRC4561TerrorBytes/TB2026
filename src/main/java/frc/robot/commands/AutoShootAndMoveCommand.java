package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class AutoShootAndMoveCommand extends Command {

    public Drive drive;
    public Indexer indexer;
    public Shooter shooter;
    public Intake intake;
    public Extension extension;
    public double distanceToHub;
    public double startTime;
    public DoubleSupplier xSupplier;
    public DoubleSupplier ySupplier;
    public Supplier<Rotation2d> angleSupplier;

    public double targetAngle;
    public double shootSpeedRPS;

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            60,
            0,
            0.6,
            new TrapezoidProfile.Constraints(8, 20)
        );

    public AutoShootAndMoveCommand(Drive drive, Indexer indexer, Shooter shooter, Extension extension, Intake intake, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        this.extension = extension;
        this.intake = intake;
        this.ySupplier = xSupplier;
        this.xSupplier = ySupplier;
        this.angleSupplier = angleSupplier;
        startTime = System.currentTimeMillis();
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive, indexer, shooter);
    }
    
    @Override
    public void initialize() {
        angleController.reset(drive.getRotation().getRadians());
    }

    @Override
    public void execute() {

        Logger.recordOutput("RobotTargetAimPose", angleSupplier.get().getDegrees());
        Logger.recordOutput("RobotTargetActualPose", drive.getRotation().getDegrees());
        
    // Construct command
              // Get linear velocity
              double x = -(xSupplier.getAsDouble()*xSupplier.getAsDouble()) * Math.signum(xSupplier.getAsDouble());
              double y = -(ySupplier.getAsDouble()*ySupplier.getAsDouble()) * Math.signum(ySupplier.getAsDouble());

              Translation2d linearVelocity =
                  DriveCommands.getLinearVelocityFromJoysticks(x, y);

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), angleSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));


        //getting hood angle from the table with interpolation
        distanceToHub = drive.getDistanceFromHubWithVelocity();
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        shooter.setHoodAngle(hoodAngleInterpolated);
        distanceToHub = drive.getDistanceToHub();
        shootSpeedRPS = shooter.getFlywheelShootSpeed(distanceToHub);
        shooter.setFlywheelSpeed(shootSpeedRPS);

        if(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint()){
            
            indexer.setThroughput(0.6, 0.7);
        }
        else{
            indexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stop();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
