// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.E_SHOOTER_POS;

public class Shooter extends SubsystemBase {
  //variables
private CANSparkMax shooterMotor1;
private CANSparkMax shooterMotor2;
  
  private Servo actuator;

  private double speedSetpoint = 0.0;
  private E_SHOOTER_POS pos;

  private double actuatorPos = 0.0;// Position 0.0 to 1.0

  //creating feedfoward and velocityPID objects
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private PIDController velPID = new PIDController(C_kP, C_kI, C_kD);

  /** Creates a new Shooter. */
  public Shooter()
  {
    shooterMotor1 = new CANSparkMax (P_SHOOTER_spMAX_1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax (P_SHOOTER_spMAX_2, MotorType.kBrushless);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    
    shooterMotor1.setInverted(true);
    shooterMotor2.follow(shooterMotor1, true);

    pos = E_SHOOTER_POS.CLOSE;

    // Actuator stuff

    actuator = new Servo(P_ACTUATOR);

    //actuator.setBounds(C_ACTUATOR_MAX_PWM_MS, 1800, ((C_ACTUATOR_MIN_PWM_MS+C_ACTUATOR_MAX_PWM_MS)/2), 1200, C_ACTUATOR_MIN_PWM_MS);
    actuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

 public void stopShooter()
 {
    shooterMotor1.stopMotor();
 }

 public void setShooterPercentage(double percent)
 {
   shooterMotor1.set(percent);
 }

 public void setShooterVoltage(double voltage)
 {
   shooterMotor1.setVoltage(voltage);
 }

 public void setShooterVelocityPID(double rpm)
 {
    this.speedSetpoint = rpm;
    double volt = 0;

    volt += feedforward.calculate(rpm / 60.0);
    volt += velPID.calculate(getShooterVel() / 60.0, rpm / 60.0);
    this.setShooterVoltage(volt);
 }

 public void setShooterVelocity(E_SHOOTER_POS pos)
 {
   if(pos == E_SHOOTER_POS.CLOSE)
   {
     this.setShooterVelocityPID(C_SHOOTER_SPEED_CLOSE);
   }
   else if (pos == E_SHOOTER_POS.FAR)
   {
     this.setShooterVelocityPID(C_SHOOTER_SPEED_FAR);
   }
   else
   {
     this.stopShooter();
   }
 }
   
 public double getShooterVel()
 {
   return shooterMotor1.getEncoder().getVelocity();
 }
 

 public boolean isFlyWheelReady()
 {
   return (Math.abs(this.getShooterVel() - this.speedSetpoint) < C_SHOOTER_SPEED_TOLERANCE);
 }
 
 // Actuator

 public void setLength(double length)// Takes in value 0.0 to 1.0
 {
   actuator.setPosition(length);
 }

 public void increaseLength()
 {
  //if (actuatorPos < 1.0) {actuatorPos += 0.2;} 
  //actuator.setPosition(actuatorPos);
  actuator.setSpeed(1);
  
 }

 public void decreaseLength()
 {
  //if (actuatorPos > 0.0) {actuatorPos -= 0.2;} 
  //actuator.setPosition(actuatorPos);
  actuator.setSpeed(-1);
 }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("speed",actuator.getSpeed());
    // This method will be called once per scheduler run
  }
}
