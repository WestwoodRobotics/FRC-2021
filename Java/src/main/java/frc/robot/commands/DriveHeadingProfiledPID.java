// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.radiansToMeters;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveHeadingProfiledPID extends ProfiledPIDCommand {
  /** Creates a new DriveHeadingProfiledPID. */
  public DriveHeadingProfiledPID(DriveTrain s_driveTrain, double degrees, double maxVel, double maxAccel) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
          0.0, 0.0, 0.0,                                          // The PID gains
          new TrapezoidProfile.Constraints(                                         // The motion profile constraints
                          radiansToMeters(maxVel),       // Max Velocity
                          radiansToMeters(maxAccel)      // Max Acceleration
                        )
        ),
        s_driveTrain::getHeadingRadians,                                            // This should return the measurement
        () -> new TrapezoidProfile.State(Math.toRadians(degrees), 0),               // This should return the goal (can also be a constant)
        
        (output, setpoint) -> {                                                     // This uses the output

          ChassisSpeeds chassisSpeeds               = new ChassisSpeeds(0, 0, setpoint.velocity + output);
          DifferentialDriveWheelSpeeds wheelSpeeds  = s_driveTrain.getKinematics().toWheelSpeeds(chassisSpeeds);
          s_driveTrain.setVelocityPID(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        }
      );

    addRequirements(s_driveTrain);
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Math.toRadians(0.5), 0.2);

    SmartDashboard.putNumber("robotAngle", degrees);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
