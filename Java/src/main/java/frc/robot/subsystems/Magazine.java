// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.MagazineConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
  
  //Motor controllers of Magazine
  private final WPI_TalonSRX feed = new WPI_TalonSRX(P_MAGAZINE_talSRX_2);
  private final WPI_TalonSRX shift = new WPI_TalonSRX(P_MAGAZINE_talSRX_1);
  
  /** Creates a new Magazine. */
  public Magazine() {

    shift.setInverted(true);
    feed.setInverted(true);
  }

  public void shiftBall(){
    // Set to an arbitrary value [-1, 1]; just needs to move the motor
    shift.set(1);
  }

  public void feedBall(){
    // Set to a variable speed
    shift.set(1);//   -1 is very important; the axis defaults at -1
    feed.set(1);
  }

  public void stopBall(){
    shift.stopMotor();
    feed.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
