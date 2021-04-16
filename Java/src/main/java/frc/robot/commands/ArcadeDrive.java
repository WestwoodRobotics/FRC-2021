// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  
  private DriveTrain s_driveTrain;
  private DoubleSupplier x;
  private DoubleSupplier y;
  
  /** Creates a new TankDrive. */
  public ArcadeDrive(DoubleSupplier x, DoubleSupplier y, DriveTrain s_driveTrain){
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_driveTrain = s_driveTrain;
    this.x = x;
    this.y = y;
    
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
    /*if(s_driveTrain.getSlowMode()){
      s_driveTrain.driveWheelsPercent(0.75*leftSpeed.getAsDouble(), 0.75*rightSpeed.getAsDouble());
    }
    else if(!s_driveTrain.getSlowMode()){
      s_driveTrain.driveWheelsPercent(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
    }*/
    s_driveTrain.driveWheelsArcade(x.getAsDouble(), y.getAsDouble());
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
