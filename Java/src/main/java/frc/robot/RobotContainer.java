// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveDistanceProfiledPID;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Subsystems
  private final DriveTrain s_driveTrain;

  // Commands


  // Joysticks
  //private final Joystick joy = new Joystick(0); 
  private final Joystick leftJoy = new Joystick(1);
  private final Joystick rightJoy = new Joystick(2);

  private final JoystickButton rightTrig = new JoystickButton(rightJoy, 1);
  
  //private final JoystickButton rightTrig = new JoystickButton(joy, 8);
  //private final Joystick rightJoy = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_driveTrain = new DriveTrain();
    
    s_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> -leftJoy.getY(),
        () -> rightJoy.getY(),
        s_driveTrain
      )
    );
    
    // s_driveTrain.setDefaultCommand(
    //   new TankDrive(
    //     () -> -leftJoy.getY(),
    //     () -> rightJoy.getY(),
    //     s_driveTrain
    //   )
    // );

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      
    //rightTrig.whenPressed(() -> s_driveTrain.toggleSlowMode());
    rightTrig.whenPressed(new RunCommand(() -> s_driveTrain.setVelocityPID(.5, .5)).withTimeout(8).andThen(() -> s_driveTrain.stopWheels(), s_driveTrain));
      (new JoystickButton(rightJoy, 2)).whenActive(new InstantCommand(() -> s_driveTrain.config()));
      //rightTrig.whenPressed(() -> s_driveTrain.setSlowMode(true)).whenReleased(() -> s_driveTrain.setSlowMode(false));
      //rightTrig.whenPressed(() -> s_driveTrain.setVelocityPID(.5, .5));

  } 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
