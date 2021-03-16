// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants. *;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  //variables
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;

  //creating feedfoward and velocityPID objects
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private PIDController velPID = new PIDController(C_kP, C_kI, C_kD);

  /** Creates a new Shooter. */
  public Shooter() 
  {
    shooterMotor1 = new CANSparkMax (P_SHOOTER_spMAX_1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax (P_SHOOTER_spMAX_2, MotorType.kBrushless);
  }

 public void stopShooter()
 {
    shooterMotor1.stopMotor();
 }

 public void setShooterPercentage(double percent)
 {
   shooterMotor1.set(percent);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
