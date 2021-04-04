// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    
    // Motor Controllers for DriveTrain
    private final WPI_TalonFX leftMaster = new WPI_TalonFX(P_DRIVE_LEFT_MASTER);
    private final WPI_TalonFX rightMaster = new WPI_TalonFX(P_DRIVE_RIGHT_MASTER);
    private final WPI_TalonFX leftFollow = new WPI_TalonFX(P_DRIVE_LEFT_FOLLOW);
    private final WPI_TalonFX rightFollow = new WPI_TalonFX(P_DRIVE_RIGHT_FOLLOW);
    
    // DifferentialDrive class for motor safety and control
    private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
    //private final SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(C_kS_RIGHT, C_kV_RIGHT, C_kA_RIGHT);

    private final PIDController leftController = new PIDController(C_kP_LEFT, C_kI_LEFT, C_kD_LEFT);
    private final PIDController rightController = new PIDController(C_kP_RIGHT, C_kI_RIGHT, C_kD_RIGHT);

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveOdometry fieldOdometry;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(C_TRACK_WIDTH_METERS);

    private AHRS imu = new AHRS();

    private boolean slowMode = false;
    private boolean straightMode = false;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        config();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeadingDegrees()));
        fieldOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeadingDegrees()));
    }

    // Drives wheels at percentage from [-1.0, +1.0]
    public void driveWheelsPercent(double leftPercent, double rightPercent){
        drive.tankDrive(leftPercent, rightPercent, true);
    }

    // Drives wheels at voltage
    public void driveWheelsVolts(double leftVolts, double rightVolts){
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        drive.feed();
    }

    public void setLeft(double leftMetersPerSec){
        SmartDashboard.putNumber("ticks per hun", metersToTicks(leftMetersPerSec)/10);
        leftMaster.set(ControlMode.Velocity, metersToTicks(leftMetersPerSec)/10);
    }

    public void setRight(double rightMetersPerSec){
        leftMaster.set(ControlMode.Velocity, metersToTicks(rightMetersPerSec)/10);
    }

    public void setVelocityPID(double leftMetersPerSec, double rightMetersPerSec){
        double leftVolts = 0;
        double rightVolts = 0;
        
        DifferentialDriveWheelSpeeds wheelSpeeds = this.getWheelSpeeds();

        leftVolts += feedforward.calculate(leftMetersPerSec);
        rightVolts += feedforward.calculate(rightMetersPerSec);

        leftVolts += leftController.calculate(wheelSpeeds.leftMetersPerSecond, leftMetersPerSec);
        rightVolts += rightController.calculate(wheelSpeeds.rightMetersPerSecond, rightMetersPerSec);

        SmartDashboard.putNumber("volts", leftMaster.getMotorOutputVoltage());
        
        drive.feed();
        this.driveWheelsVolts(leftVolts, rightVolts);
        
    }

    public double distanceFromGoal(){
        return this.getFieldPose().getTranslation().getDistance(C_GOAL_POSE);
    }

    public double angleToGoalDeg(){
        double diffX = this.getFieldPose().getTranslation().getX() - C_GOAL_POSE.getX();
        double diffY = -(this.getFieldPose().getTranslation().getY() + C_GOAL_POSE.getY());
        return Math.toDegrees(Math.tan(diffX/diffY));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftEncoderVelMeters(), rightEncoderVelMeters());
        return speeds;
    }

    public void config(){
        rightMaster.setInverted(true);
        rightFollow.setInverted(true);
        
        leftMaster.setInverted(false);
        leftFollow.setInverted(false);

        leftFollow.follow(leftMaster);
        rightFollow.follow(rightMaster);

        drive.setSafetyEnabled(true);
        
        zeroLeftEncoder();
        zeroRightEncoder();

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        resetFieldOdometry();
    }

    public void stopWheels(){
        drive.stopMotor();
        leftMaster.stopMotor();
        rightMaster.stopMotor();
    }

    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }
    public boolean getSlowMode(){
        return slowMode; 
    }

    public void toggleSlowMode(){
        this.setSlowMode(!this.getSlowMode());
    }


    public void setStraightMode(boolean straightMode) {
        this.straightMode = straightMode;
    }
    public boolean getStraightMode(){
        return straightMode; 
    }

    public void toggleStraightMode(){
        this.setStraightMode(!this.getStraightMode());
    }
    

    // Encoder functions

    // Ticks
    public double leftEncoderGetTicks(){return leftMaster.getSelectedSensorPosition();}
    public double rightEncoderGetTicks(){return rightMaster.getSelectedSensorPosition();}
    
    // Meters
    public double leftEncoderGetMeters(){return ticksToMeters(leftEncoderGetTicks());}
    public double rightEncoderGetMeters(){return ticksToMeters(rightEncoderGetTicks());}
    
    // Ticks per 100 ms
    public double leftEncoderVelTicks(){return leftMaster.getSelectedSensorVelocity();}
    public double rightEncoderVelTicks(){return rightMaster.getSelectedSensorVelocity();}

    // Meters per s
    public double leftEncoderVelMeters(){return ticksToMeters(leftEncoderVelTicks())*10;}
    public double rightEncoderVelMeters(){return ticksToMeters(rightEncoderVelTicks())*10;}

    // Average of both wheels
    public double getAverageEncoderTicks()    {return (leftEncoderGetTicks() + rightEncoderGetTicks())/2.0;}
    public double getAverageEncoderMeters()   {return (leftEncoderGetMeters() + rightEncoderGetMeters())/2.0;}
    public double getAverageEncoderVelMeters()      {return (leftEncoderVelMeters() + rightEncoderVelMeters())/2.0;}

    // Reset
    public void zeroLeftEncoder(){leftMaster.setSelectedSensorPosition(0);}
    public void zeroRightEncoder(){rightMaster.setSelectedSensorPosition(0);}

    // Gyro functions
    public double getHeadingDegrees()         {return -imu.getYaw();}
    public double getHeadingRadians()         {return Math.toRadians(this.getHeadingDegrees());}
    public double getTurnRate()               {return imu.getRate();} 
    public void   zeroHeading()               {imu.reset();}
    
    // Odometry
    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(pose, Rotation2d.fromDegrees(this.getHeadingDegrees()));
    }

    public void resetFieldOdometry(){
        zeroLeftEncoder();
        zeroRightEncoder();

        zeroHeading();
    
        odometry.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), Rotation2d.fromDegrees(this.getHeadingDegrees()));
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public Pose2d getFieldPose(){
        return fieldOdometry.getPoseMeters();
    }

    public Command getTrajectoryCommand(double maxVel, double maxAccel, Pose2d initialPose, List interiorWaypoints, Pose2d endPose){
        resetOdometry(initialPose);
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, kinematics, C_MAX_VOLTAGE);

        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel).setKinematics(kinematics).addConstraint(autoVoltageConstraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(initialPose, interiorWaypoints, endPose, config);

        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory, 
                this::getPose, 
                new RamseteController(C_kB_RAMSETE, C_kZ_RAMSETE), 
                feedforward, 
                kinematics, 
                this::getWheelSpeeds, 
                leftController, 
                rightController, 
                this::driveWheelsVolts, 
                this
            );

        
        return ramseteCommand;
    }

    public Command getTrajectoryCommand(double maxVel, double maxAccel, Trajectory trajectory){
        resetOdometry(trajectory.getInitialPose());
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, kinematics, C_MAX_VOLTAGE);

        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel).setKinematics(kinematics).addConstraint(autoVoltageConstraint);

        RamseteCommand ramseteCommand = new RamseteCommand(
                trajectory, 
                this::getPose, 
                new RamseteController(C_kB_RAMSETE, C_kZ_RAMSETE), 
                feedforward, 
                kinematics, 
                this::getWheelSpeeds, 
                leftController, 
                rightController, 
                this::driveWheelsVolts, 
                this
            );

        
        return ramseteCommand;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left encoder", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("right encoder", rightMaster.getSelectedSensorPosition());
        
        SmartDashboard.putNumber("left meters", ticksToMeters(leftMaster.getSelectedSensorPosition()));
        SmartDashboard.putNumber("right meters", ticksToMeters(rightMaster.getSelectedSensorPosition()));        
        
        SmartDashboard.putNumber("leftvel", this.leftEncoderVelMeters());
        SmartDashboard.putNumber("rightvel", this.rightEncoderVelMeters());

        odometry.update(
                Rotation2d.fromDegrees(this.getHeadingDegrees()), 
                this.leftEncoderGetMeters(), 
                this.rightEncoderGetMeters()
              );

        fieldOdometry.update(
                Rotation2d.fromDegrees(this.getHeadingDegrees()), 
                this.leftEncoderGetMeters(), 
                this.rightEncoderGetMeters()
              );

        drive.feed();

        SmartDashboard.putNumber("x", fieldOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", fieldOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", fieldOdometry.getPoseMeters().getRotation().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
