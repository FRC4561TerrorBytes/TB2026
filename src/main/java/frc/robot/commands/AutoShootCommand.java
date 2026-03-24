package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

    public AutoShootCommand(Drive drive, DoubleSupplier X, DoubleSupplier Y, Indexer indexer, Shooter shooter) {
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(drive, indexer, shooter);

        joystickX = X;
        joystickY = Y;

        controller = new PIDController(0.1, 0, 0, 0.02);
        controller.setTolerance(1.5);
        controller.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        //distanceToHub = drive.getPose().getTranslation().getDistance(AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
        distanceToHub = drive.getDistanceToHub();
        shootSpeedRPS = shooter.getFlywheelShootSpeed(distanceToHub);
        shooter.setFlywheelSpeed(shootSpeedRPS);

        //getting hood angle from the table with interpolation
        double hoodAngleInterpolated = shooter.interpolateHoodAngle(distanceToHub);
        shooter.setHoodAngle(hoodAngleInterpolated);

        double targetAngle = drive.getRotationToHub().getDegrees();
        Logger.recordOutput("TargetAngleToFace", targetAngle);

        double rotaionSpeed = MathUtil.clamp(controller.calculate(drive.getPose().getRotation().getDegrees(), targetAngle), -30, 30);
        double linearVelocity = Math.sqrt(Math.pow(joystickX.getAsDouble(), 2) + Math.pow(joystickY.getAsDouble(), 2));

        if(!controller.atSetpoint()){
            drive.runVelocity(new ChassisSpeeds(-joystickX.getAsDouble(), -joystickY.getAsDouble(), rotaionSpeed));
        }
        else if(linearVelocity < 0.1){
            drive.stopWithX();
        }
        else{
            drive.runVelocity(new ChassisSpeeds(-joystickX.getAsDouble(), -joystickY.getAsDouble(), 0));
        }

        if(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint() && controller.atSetpoint() && linearVelocity < 0.1){
            indexer.setThroughput(0.6, 0.7);
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
