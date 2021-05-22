// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveHeadingProfiled extends TrapezoidProfileCommand {
  /** Creates a new DriveHeadingProfiled. */
  public DriveHeadingProfiled(DriveTrain s_driveTrain, double degrees, double maxVel, double maxAccel) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(maxVel, maxAccel),
            // Goal state
            new TrapezoidProfile.State(degrees, 0.0),
            // Initial state
            new TrapezoidProfile.State(0.0, 0.0)),
        state -> {
          s_driveTrain.setVelocityPID(state.velocity, -state.velocity);
          // Use current trajectory state here
        });

    addRequirements(s_driveTrain);
  }
}
