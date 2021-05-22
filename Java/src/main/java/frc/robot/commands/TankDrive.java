// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  
  private DriveTrain s_driveTrain;
  private DoubleSupplier leftSpeed;
  private DoubleSupplier rightSpeed;
  
  /** Creates a new TankDrive. */
  public TankDrive(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DriveTrain s_driveTrain){
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_driveTrain = s_driveTrain;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    
    addRequirements(s_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_driveTrain.stopWheels();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeedD = leftSpeed.getAsDouble();
    double rightSpeedD = s_driveTrain.getStraightMode()?-leftSpeed.getAsDouble():rightSpeed.getAsDouble();

    if(s_driveTrain.getSlowMode()){
      leftSpeedD *= .75;
      rightSpeedD *= .75;
    }

    s_driveTrain.driveWheelsPercent(leftSpeedD, rightSpeedD);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_driveTrain.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
