
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
import edu.wpi.first.wpilibj2.command.Commands;
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
    public double distanceToHub;

    public double targetAngle;
    public double shootSpeedRPS;

    public AutoShootAndMoveCommand(Drive drive, Indexer indexer, Shooter shooter) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {

        Logger.recordOutput("RobotTargetAimPose",drive.getRotationToHubWithVelocity().getDegrees());
        Logger.recordOutput("RobotTargetActualPose", drive.getRotation().getDegrees());

        distanceToHub = drive.getDistanceFromHubWithVelocity();

        //getting hood angle from the table with interpolation
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
