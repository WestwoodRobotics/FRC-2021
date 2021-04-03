// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.RunPaths;
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

  // Subsystems
  private final DriveTrain s_driveTrain;
  //private final Magazine s_magazine;
  // Commands

  private Command barrelPath;
  private Command bouncePath;
  private Command blueA;
  private Command redA;
  private Command blueB;
  private Command redB;
  private Command slalom;

  // Joysticks
  //private final Joystick joy = new Joystick(0); 
  private final Joystick leftJoy = new Joystick(1);
  private final Joystick rightJoy = new Joystick(2);

  // Y-box controller
  private final Joystick mechJoy = new Joystick(3);

  // Configure joystick buttons
  private final JoystickButton rightTrig = new JoystickButton(rightJoy, 1);
  
  // Y-box controller triggers, bumpers, buttons
  private final JoystickButton mechRightBumper = new JoystickButton(mechJoy, 6);
  private final JoystickButton mechLeftTrigger = new JoystickButton(mechJoy, 7);

  //private final JoystickButton rightTrig = new JoystickButton(joy, 8);
  //private final Joystick rightJoy = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_driveTrain = new DriveTrain();
    //s_magazine = new Magazine();

    s_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> -leftJoy.getY(),
        () -> rightJoy.getY(),
        s_driveTrain
      )
    );

    /*s_magazine.setDefaultCommand(
      new RunCommand(
        () -> {
          if (mechLeftTrigger.get()){
            s_magazine.shiftBall();
          }
          else{
            s_magazine.feedBall(() -> mechJoy.getRawAxis(4));
          }
        },
        s_magazine
      )
    );*/
    
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
      
    rightTrig.whenPressed(() -> s_driveTrain.toggleSlowMode());
    rightTrig.whenPressed(new RunCommand(() -> s_driveTrain.setVelocityPID(0.5, 0.5)));
    //rightTrig.whenPressed(new DriveDistanceProfiledPID(s_driveTrain, 5, 0, 1, 1));
    (new JoystickButton(rightJoy, 2)).whenActive(new InstantCommand(() -> s_driveTrain.config()));

    //mechLeftTrigger.whenPressed(() -> s_magazine.shiftBall()).whenReleased(() -> s_magazine.stopBall());
  } 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return barrelPath;
  }

  public void loadBarrel(){

    String barrel1File = "paths/BarrelTest1.wpilib.json";


    Trajectory barrel1 = new Trajectory();
    //Trajectory barrel2 = new Trajectory();
    Command barrel2 = s_driveTrain.getTrajectoryCommand(
      3, 
      1.75, 
      new Pose2d(6.117, -2.199, new Rotation2d(0.0)), 
      List.of(
      null
      ), 
      new Pose2d(1.289, -1.87, new Rotation2d(0.0)));

    try {
      Path barrel1Path = Filesystem.getDeployDirectory().toPath().resolve(barrel1File);
      barrel1 = TrajectoryUtil.fromPathweaverJson(barrel1Path);


    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + barrel1File, ex.getStackTrace());
    }
    
    barrelPath = new RunPaths(s_driveTrain, List.of(barrel1)).andThen(barrel2);
    
  }

  public void loadGalacticSearch(){
    String pathABlueStr = "paths/PathABlue.wpilib.json";
    String pathARedStr = "paths/PathARed.wpilib.json";
    String pathBBlueStr = "paths/PathBBlue.wpilib.json";
    String pathBRedStr = "paths/PathBRed.wpilib.json";

    Trajectory pathABlueTraj = new Trajectory();
    Trajectory pathARedTraj = new Trajectory();
    Trajectory pathBBlueTraj = new Trajectory();
    Trajectory pathBRedTraj = new Trajectory();

    try {
      Path pathABlue = Filesystem.getDeployDirectory().toPath().resolve(pathABlueStr);
      pathABlueTraj = TrajectoryUtil.fromPathweaverJson(pathABlue);

      Path pathARed = Filesystem.getDeployDirectory().toPath().resolve(pathARedStr);
      pathARedTraj = TrajectoryUtil.fromPathweaverJson(pathARed);

      Path pathBBlue = Filesystem.getDeployDirectory().toPath().resolve(pathBBlueStr);
      pathBBlueTraj = TrajectoryUtil.fromPathweaverJson(pathBBlue);
      
      Path pathBRed = Filesystem.getDeployDirectory().toPath().resolve(pathBRedStr);
      pathBRedTraj = TrajectoryUtil.fromPathweaverJson(pathBRed);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathABlueStr, ex.getStackTrace());
    }
    
    blueA = new RunPaths(s_driveTrain, List.of(pathABlueTraj));
    blueB = new RunPaths(s_driveTrain, List.of(pathBBlueTraj));
    redA = new RunPaths(s_driveTrain, List.of(pathARedTraj));
    redB = new RunPaths(s_driveTrain, List.of(pathBRedTraj));


  }

  public void loadBounce(){
    String bounce1File = "paths/BounceTest1.wpilib.json";
    String bounce2File = "paths/BounceTest2.wpilib.json";
    String bounce3File = "paths/BounceTest3.wpilib.json";
    String bounce4File = "paths/BounceTest4.wpilib.json";

    Trajectory bounce1 = new Trajectory();
    Trajectory bounce2 = new Trajectory();
    Trajectory bounce3 = new Trajectory();
    Trajectory bounce4 = new Trajectory();

    try {
      Path bounce1Path = Filesystem.getDeployDirectory().toPath().resolve(bounce1File);
      bounce1 = TrajectoryUtil.fromPathweaverJson(bounce1Path);

      Path bounce2Path = Filesystem.getDeployDirectory().toPath().resolve(bounce2File);
      bounce2 = TrajectoryUtil.fromPathweaverJson(bounce2Path);

      Path bounce3Path = Filesystem.getDeployDirectory().toPath().resolve(bounce3File);
      bounce3 = TrajectoryUtil.fromPathweaverJson(bounce3Path);

      Path bounce4Path = Filesystem.getDeployDirectory().toPath().resolve(bounce4File);
      bounce4 = TrajectoryUtil.fromPathweaverJson(bounce4Path);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + bounce1File, ex.getStackTrace());
    }

    bouncePath = new RunPaths(s_driveTrain, List.of(
      bounce1,
      bounce2,
      bounce3,
      bounce4
    ));
  }

  public void loadSlalom(){
    String trajectoryJSON = "paths/SlalomPath.wpilib.json";
    Trajectory slalomTraj = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      slalomTraj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    
    slalom = new RunPaths(s_driveTrain, List.of(slalomTraj));
    
  }


}
