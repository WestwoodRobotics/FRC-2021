// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceProfiledPID extends ProfiledPIDCommand {
  /** Creates a new DriveDistanceProfiledPID. */
  public DriveDistanceProfiledPID(DriveTrain s_driveTrain, double meters, double endVelocity, double maxVel, double maxAccel) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            3.0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(maxVel, maxAccel)),
        // This should return the measurement
        () -> s_driveTrain.getAverageEncoderMeters(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(meters, endVelocity),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          s_driveTrain.setVelocityPID(output, output);
          SmartDashboard.putNumber("output", output);
        }, s_driveTrain);

    getController().disableContinuousInput();
    getController().setTolerance(0.1);
    addRequirements(s_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
