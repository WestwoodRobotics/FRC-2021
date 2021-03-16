// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  
  //Motor controllers of Magazine
  private final WPI_VictorSPX feed = new WPI_VictorSPX(P_MAGAZINE_vicSPX_1);
  private final WPI_VictorSPX shift = new WPI_VictorSPX(P_MAGAZINE_vicSPX_2);
  
  /** Creates a new Magazine. */
  public Magazine() {}

  public void shiftBall(){
    // Set to an arbitrary value [-1, 1]; just needs to move the motor
    shift.set(0.5);
  }

  public void feedBall(DoubleSupplier speedSupplier){
    // Set to a variable speed
    shift.set(speedSupplier.getAsDouble());
    feed.set(speedSupplier.getAsDouble());
  }

  public void stopBall(){
    shift.stopMotor(0);
    feed.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
