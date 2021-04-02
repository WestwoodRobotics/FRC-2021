// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunPaths extends SequentialCommandGroup {
  /** Creates a new BarrelPath. */
  public RunPaths(DriveTrain s_driveTrain, List trajectories) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    //addCommands()
      /*s_driveTrain.getTrajectoryCommand(0.75, 0.5, 
      new Pose2d(-1.0, 0.0, new Rotation2d(0.0)),
      List.of(
        new Translation2d(0.0, 1.0),
        new Translation2d(1.0, 1.0),
        new Translation2d(2.0, 1.0),
        new Translation2d(2.0, 0.0),
        new Translation2d(2.0, -1.0),
        new Translation2d(1.0, -1.0),
        new Translation2d(0.0, -1.0)
      ),
      new Pose2d(0.0, 0.0, new Rotation2d(90.0)))*/
    for(Object t : trajectories){
      addCommands(
        new InstantCommand(() -> s_driveTrain.resetOdometry(((Trajectory)t).getInitialPose())),
        s_driveTrain.getTrajectoryCommand(2.25, 1.25, (Trajectory)t)
      );
    }
  }
}
