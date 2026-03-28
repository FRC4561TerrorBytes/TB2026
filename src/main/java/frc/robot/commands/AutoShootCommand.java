package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;

public class AutoShootCommand extends Command {

    public Drive drive;
    public Indexer indexer;
    public Shooter shooter;
    public double distanceToHub;

    public double targetAngle;
    public double shootSpeedRPS = 70;
    PIDController controller;
    DoubleSupplier joystickX;
    DoubleSupplier joystickY;

    public AutoShootCommand(Drive drive, Indexer indexer, Shooter shooter){
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(drive, indexer, shooter);

        controller = new PIDController(2.5, 0, 0, 0.02);
        controller.setTolerance(Units.degreesToRadians(1.5));
        controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    public AutoShootCommand(Drive drive, DoubleSupplier X, DoubleSupplier Y, Indexer indexer, Shooter shooter) {
        this(drive, indexer, shooter);
        
        joystickX = X;
        joystickY = Y;
    }

    

    @Override
    public void execute() {
        distanceToHub = drive.getDistanceToHub();
        shootSpeedRPS = shooter.getFlywheelShootSpeed(distanceToHub);
        shooter.setFlywheelSpeed(shootSpeedRPS);

        //getting hood angle from the table with interpolation
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        shooter.setHoodAngle(hoodAngleInterpolated);

        double targetAngle = drive.getRotationToHub().getDegrees();
        Logger.recordOutput("TargetAngleToFace", targetAngle);

        double rotationSpeed = MathUtil.clamp(controller.calculate(drive.getPose().getRotation().getRadians(), Units.degreesToRadians(targetAngle)), -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

        double x = (joystickY.getAsDouble() * joystickY.getAsDouble()) * Math.signum(-joystickY.getAsDouble());
        double y = (joystickX.getAsDouble() * joystickX.getAsDouble()) * Math.signum(-joystickX.getAsDouble());
        Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(x, y);

        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        
        ChassisSpeeds speeeeeeeeds = new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(), 
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), 
                rotationSpeed);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeeeeeeeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        if(Math.hypot(linearVelocity.getX(), linearVelocity.getY()) < Math.pow(0.05, 2) && controller.atSetpoint()){
            drive.stopWithX();
        }

        if(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint() && controller.atSetpoint() && Math.hypot(linearVelocity.getX(), linearVelocity.getY()) < Math.pow(0.05, 2)){
            indexer.setThroughput(0.9, 0.8);
        }
        else{
            indexer.stop();
        }

        Logger.recordOutput("DriveTrainFacingHub", controller.atSetpoint());
        Logger.recordOutput("ShooterReady", shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint());
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
