// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.MagazineConstants.P_MAGAZINE_talSRX_1;
import static frc.robot.Constants.MagazineConstants.P_MAGAZINE_talSRX_2;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  
  
  private final WPI_TalonSRX feed = new WPI_TalonSRX(P_MAGAZINE_talSRX_2);
  private final WPI_TalonSRX shift = new WPI_TalonSRX(P_MAGAZINE_talSRX_1);
  
  
  public Magazine() {

    shift.setInverted(true);
    feed.setInverted(true);

    feed.setNeutralMode(NeutralMode.Brake);
  }

  public void shiftBall(){
    
    shift.set(1);
  }

  public void feedBall(){
    
    shift.set(1);
    feed.set(1);
  }

  public void stopBall(){
    shift.stopMotor();
    feed.stopMotor();
  }


  @Override
  public void periodic() {}
}
