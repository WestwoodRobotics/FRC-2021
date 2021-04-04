// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterPose;

public class RunShooter extends CommandBase {
  /** Creates a new RunShooter. */
  private Shooter s_shooter;
  private DriveTrain s_driveTrain;

  private ShooterPose newPose;

  public RunShooter(Shooter s_shooter, DriveTrain s_driveTrain) {
    addRequirements(s_shooter, s_driveTrain);
    this.s_shooter = s_shooter;
    this.s_driveTrain = s_driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_shooter.setDegrees(s_shooter.getShooterPose().launchAngleDeg);
    s_shooter.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //newPose = new ShooterPose();
    s_shooter.setShooterPose(newPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
