package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;

public class AutoShootCommand extends Command {

    public Drive drive;
    public Indexer indexer;
    public Shooter shooter;

    public double targetAngle;
    public double shootSpeedRPS = 70;
    PIDController controller;
    DoubleSupplier joystickX;
    DoubleSupplier joystickY;

    public boolean moving = false;
    public boolean driveRotated = false;
    public boolean shooterReady = false;

    private final Debouncer driveDebouncer = new Debouncer(0.1, DebounceType.kFalling);
    private final Debouncer shooterDebouncer = new Debouncer(0.1, DebounceType.kFalling);


    public AutoShootCommand(Drive drive, Indexer indexer, Shooter shooter){
        this.drive = drive;
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(drive, indexer, shooter);

        controller = new PIDController(9, 0.0001, 0.5, 0.02);
        controller.setTolerance(Units.degreesToRadians(3.0));
        controller.enableContinuousInput(-Math.PI, Math.PI);

        joystickX = () -> 0.0;
        joystickY = () -> 0.0;

        Leds.getInstance().autoScoreAtRotationSetpoint = false;
        Leds.getInstance().autoScoreRotatePercent = 0.0;
    }

    public AutoShootCommand(Drive drive, DoubleSupplier X, DoubleSupplier Y, Indexer indexer, Shooter shooter) {
        this(drive, indexer, shooter);
        
        joystickX = X;
        joystickY = Y;
    }

    

    @Override
    public void execute() {
        //setting targets for flywheel and hood
        shooter.setFlywheelSpeed(shooter.getFlywheelShootSpeed(drive.getDistanceToHub()));
        shooter.setHoodAngle(shooter.interpolateHoodAngle(drive.getDistanceToHub()));

        //rotation calculations for drive
        double targetAngle = drive.getRotationToHub().getRadians();
        double rotationSpeed = MathUtil.clamp(controller.calculate(drive.getPose().getRotation().getRadians(), targetAngle), -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

        //joystick inputs to keep robot moving
        double x = (joystickY.getAsDouble() * joystickY.getAsDouble()) * Math.signum(-joystickY.getAsDouble());
        double y = (joystickX.getAsDouble() * joystickX.getAsDouble()) * Math.signum(-joystickX.getAsDouble());
        Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(x, y);

        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        
        //creating chassis speeds using both joystick and rotation to hub controller
        ChassisSpeeds speeeeeeeeds = new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(), 
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), 
                rotationSpeed);

        //setting varaibles used to determine actions of the robot
        driveRotated = driveDebouncer.calculate(controller.atSetpoint());
        shooterReady = shooterDebouncer.calculate(shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint());
        moving = Math.hypot(linearVelocity.getX(), linearVelocity.getY()) > 0.3;

        if(moving || !controller.atSetpoint()){
            drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeeeeeeeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        }

        if(!moving && driveRotated){
            drive.stopWithX();
        }

        if(shooterReady && driveRotated && !moving){
            indexer.setThroughput(0.9, 1.0);
        }
        else{
            indexer.stop();
        }

        Leds.getInstance().autoScoreAtRotationSetpoint = controller.atSetpoint();
        Leds.getInstance().autoScoreRotatePercent = 1.0 - (Math.abs(controller.getError())/Math.PI);

        Logger.recordOutput("AutoShoot/TargetAngleToFace", targetAngle);
        Logger.recordOutput("AutoShoot/linearVelocity", Math.hypot(linearVelocity.getX(), linearVelocity.getY()));
        Logger.recordOutput("AutoShoot/RAWDriveTrainFacingHub", controller.atSetpoint());
        Logger.recordOutput("AutoShoot/DEBOUNCEDDriveTrainFacingHub", driveRotated);
        Logger.recordOutput("AutoShoot/RAWShooterReady", shooter.leftFlywheelUpToSpeed(shootSpeedRPS) && shooter.rightFlywheelUpToSpeed(shootSpeedRPS) && shooter.hoodAtSetpoint());
        Logger.recordOutput("AutoShoot/DEBOUNCEDShooterReady", shooterReady);
    }

    @Override
    public void end(boolean interrupted) {
        Leds.getInstance().autoScoreAtRotationSetpoint = false;
        Leds.getInstance().autoScoreRotatePercent = 0.0;
        shooter.stop();
        indexer.stop();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
