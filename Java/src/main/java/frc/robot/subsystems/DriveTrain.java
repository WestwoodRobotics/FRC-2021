// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    
    // Motor Controllers for DriveTrain
    private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(P_DRIVE_LEFT_MASTER_talSRX);
    private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(P_DRIVE_RIGHT_MASTER_talSRX);
    private final WPI_VictorSPX leftFollow = new WPI_VictorSPX(P_DRIVE_LEFT_FOLLOW_vicSPX);
    private final WPI_VictorSPX rightFollow = new WPI_VictorSPX(P_DRIVE_RIGHT_FOLLOW_vicSPX);
    
    // DifferentialDrive class for motor safety and control
    private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
    private final SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(C_kS_LEFT, C_kV_LEFT, C_kA_LEFT);
    private final SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(C_kS_RIGHT, C_kV_RIGHT, C_kA_RIGHT);

    private final PIDController leftController = new PIDController(C_kP_LEFT, C_kI_LEFT, C_kD_LEFT);
    private final PIDController rightController = new PIDController(C_kP_RIGHT, C_kI_RIGHT, C_kD_RIGHT);

    private boolean slowMode = false;

    /** Creates a new ExampleSubsystem. */
    public DriveTrain() {
        config();
    }

    // Drives wheels at percentage from [-1.0, +1.0]
    public void driveWheelsPercent(double leftPercent, double rightPercent){
        drive.tankDrive(0.9*leftPercent, rightPercent);
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

        leftVolts += feedforwardLeft.calculate(leftMetersPerSec);
        rightVolts += feedforwardRight.calculate(rightMetersPerSec);

        leftVolts += leftController.calculate(wheelSpeeds.leftMetersPerSecond, leftMetersPerSec);
        rightVolts += rightController.calculate(wheelSpeeds.rightMetersPerSecond, rightMetersPerSec);

        //SmartDashboard.putNumber("volts", leftMaster.getMotorOutputVoltage());

        this.driveWheelsVolts(leftVolts, rightVolts);

        //SmartDashboard.putNumber("vel", (leftEncoderVelMeters()+rightEncoderVelMeters())/2);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftEncoderVelMeters(), rightEncoderVelMeters());
        return speeds;
    }

    public void config(){
        rightMaster.setInverted(true);
        rightFollow.setInverted(true);
        
        leftFollow.follow(leftMaster);
        rightFollow.follow(rightMaster);
        
        drive.setSafetyEnabled(true);
        
        zeroLeftEncoder();
        zeroRightEncoder();

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        
        //leftMaster.config_kP(0, 0);
        //leftMaster.config_kI(0, C_kI_LEFT);
        //leftMaster.config_kD(0, C_kD_LEFT);
        //leftMaster.config_kF(0, 0);
        
        //rightMaster.config_kP(0, 0);
        //rightMaster.config_kI(0, C_kI_RIGHT);
        //rightMaster.config_kD(0, C_kD_RIGHT);
        //rightMaster.config_kF(0, 0);
        
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

    // Ticks
    public double leftEncoderGetTicks(){return leftMaster.getSelectedSensorPosition();}
    public double rightEncoderGetTicks(){return rightMaster.getSelectedSensorPosition();}
    
    // Meters
    public double leftEncoderGetMeters(){return ticksToMeters(leftMaster.getSelectedSensorPosition());}
    public double rightEncoderGetMeters(){return ticksToMeters(rightMaster.getSelectedSensorPosition());}
    
    // Ticks per 100 ms
    public double leftEncoderVelTicks(){return leftMaster.getSelectedSensorVelocity();}
    public double rightEncoderVelTicks(){return rightMaster.getSelectedSensorVelocity();}

    // Meters per s
    public double leftEncoderVelMeters(){return ticksToMeters(leftMaster.getSelectedSensorVelocity())*10;}
    public double rightEncoderVelMeters(){return ticksToMeters(rightMaster.getSelectedSensorVelocity())*10;}

    public double getAverageEncoderTicks()    {return (leftEncoderGetTicks() + rightEncoderGetTicks())/2.0;}
    public double getAverageEncoderMeters()   {return (leftEncoderGetMeters() + rightEncoderGetMeters())/2.0;}
    public double getAverageEncoderVelMeters()      {return (leftEncoderVelMeters() + rightEncoderVelMeters())/2.0;}

    public void zeroLeftEncoder(){leftMaster.setSelectedSensorPosition(0);}
    public void zeroRightEncoder(){rightMaster.setSelectedSensorPosition(0);}

    @Override
    public void periodic() {
        //driveWheelsPercent(.5, .5);
        SmartDashboard.putNumber("right encoder", rightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("left encoder", leftMaster.getSelectedSensorPosition());

        SmartDashboard.putNumber("right meters", ticksToMeters(rightMaster.getSelectedSensorPosition()));
        SmartDashboard.putNumber("left meters", ticksToMeters(leftMaster.getSelectedSensorPosition()));
        //leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        //rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
