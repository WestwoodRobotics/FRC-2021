// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunShooter;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final DriveTrain s_driveTrain;
  private final Magazine s_magazine;
  private final Shooter s_shooter;

  // Commands
  

  // Joysticks
  //private final Joystick joy = new Joystick(0); 
  private final Joystick leftJoy = new Joystick(1);
  private final Joystick rightJoy = new Joystick(2);

  // Y-box controller
  private final Joystick mechJoy = new Joystick(3);

  // Configure joystick buttons
  private final JoystickButton rightTrig = new JoystickButton(rightJoy, 1);
  
  // Y-box controller triggers, bumpers, buttons
  //private final JoystickButton mechRightBumper = new JoystickButton(mechJoy, 6);
  private final JoystickButton mechRightTrigger = new JoystickButton(mechJoy, 8);
  private final JoystickButton mechLeftTrigger = new JoystickButton(mechJoy, 7);

  private final JoystickButton mechTriangle = new JoystickButton(mechJoy, 4);
  private final JoystickButton mechCircle = new JoystickButton(mechJoy, 3);
  private final JoystickButton mechSquare = new JoystickButton(mechJoy, 1);
  private final JoystickButton mechCross = new JoystickButton(mechJoy, 2);

  //private final JoystickButton rightTrig = new JoystickButton(joy, 8);
  //private final Joystick rightJoy = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_driveTrain = new DriveTrain();
    s_magazine = new Magazine();

    s_shooter = new Shooter();
  

    s_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> -leftJoy.getY(),
        () -> rightJoy.getY(),
        s_driveTrain
      )
    );
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      
    rightTrig.whenPressed(() -> s_driveTrain.toggleSlowMode());
    //rightTrig.whenPressed(new RunCommand(() -> s_driveTrain.setVelocityPID(0.5, 0.5)));
    //rightTrig.whenPressed(new DriveDistanceProfiledPID(s_driveTrain, 5, 0, 1, 1));
    (new JoystickButton(rightJoy, 2)).whenActive(new InstantCommand(() -> s_driveTrain.config()));

    mechTriangle.whenPressed(() -> s_shooter.increaseLength());
    mechCircle.whenPressed(()-> s_shooter.decreaseLength());

    mechLeftTrigger.whenPressed(() -> s_magazine.shiftBall()).whenReleased(() -> s_magazine.stopBall());
    mechRightTrigger.whenPressed(() -> s_magazine.feedBall()).whenReleased(() -> s_magazine.stopBall());

    mechSquare.toggleWhenPressed(new RunShooter(s_shooter, 2000));
    //mechSquare.toggleWhenPressed(new StartEndCommand(new RunCommand(() -> s_shooter.setShooterVelocityPID(4000)), () -> s_shooter.stopShooter(), s_shooter));

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
