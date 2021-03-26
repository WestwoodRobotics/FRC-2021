// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.MagazineConstants. *;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  
  //Motor controllers of Magazine
  //private final WPI_VictorSPX feed = new WPI_VictorSPX(P_MAGAZINE_vicSPX_1);
  private final WPI_TalonSRX shift = new WPI_TalonSRX(P_MAGAZINE_talSRX_2);
  
  /** Creates a new Magazine. */
  public Magazine() {

    shift.setInverted(true);
  }

  public void shiftBall(){
    // Set to an arbitrary value [-1, 1]; just needs to move the motor
    shift.set(0.5);
  }

  public void feedBall(DoubleSupplier speedSupplier){
    // Set to a variable speed
    shift.set((speedSupplier.getAsDouble()+1.0)/2);//   -1 is very important; the axis defaults at -1
    //feed.set(speedSupplier.getAsDouble());
  }

  public void stopBall(){
    shift.stopMotor();
    //feed.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
