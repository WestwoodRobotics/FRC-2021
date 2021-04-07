// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // variables
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;

  private Servo actuator;

  private ShooterPose currentPose = new ShooterPose(C_ACTUATOR_MAX_DEG, 0.0);

  // creating feedfoward and velocityPID objects
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(C_kS, C_kV, C_kA);
  private PIDController velPID = new PIDController(C_kP, C_kI, C_kD);

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor1 = new CANSparkMax(P_SHOOTER_spMAX_1, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(P_SHOOTER_spMAX_2, MotorType.kBrushless);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor2.setIdleMode(IdleMode.kCoast);

    shooterMotor1.setInverted(true);
    shooterMotor2.follow(shooterMotor1, true);

    // Actuator stuff

    actuator = new Servo(P_ACTUATOR);

    // Sets the maximum, center, and minimum values of the PWM pulse (2, 1.5, and 1 seconds) as defined in
    // the actuator datasheet. Also deals with deadband (1.8 and 1.2)(not crucial with limited PWM devices)
    actuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  public void stopShooter() {
    shooterMotor1.stopMotor();
    this.currentPose.setLaunchRPM(0.0);
  }

  public void setShooterPercentage(double percent) {
    shooterMotor1.set(percent);
  }

  public void setShooterVoltage(double voltage) {
    shooterMotor1.setVoltage(voltage);
  }

  public void setShooterVelocityPID(double rpm) {
    this.currentPose.setLaunchRPM(rpm);
    double volt = 0;

    volt += feedforward.calculate(rpm / 60.0);
    SmartDashboard.putNumber("volt feedforward", volt);
    volt += velPID.calculate(getShooterVel() / 60.0, rpm / 60.0);
    SmartDashboard.putNumber("volt velocityPID", volt);

    this.setShooterVoltage(volt);
  }

  /*public void setShooterVelocity(E_SHOOTER_POS pos) {
    if (pos == E_SHOOTER_POS.CLOSE) {
      this.setShooterVelocityPID(C_SHOOTER_SPEED_CLOSE);
    } else if (pos == E_SHOOTER_POS.FAR) {
      this.setShooterVelocityPID(C_SHOOTER_SPEED_FAR);
    } else {
      this.stopShooter();
    }
  }*/

  public double getShooterVel() {
    return shooterMotor1.getEncoder().getVelocity();
  }

  // Actuator
  public void increaseLength()// Used for testing
  {
    if (this.currentPose.launchAngleExtension < C_ACTUATOR_MAX_DEG) {
      this.currentPose.launchAngleDeg += 3;
    }
    // actuator.setPosition(actuatorPos);
    // actuator.setSpeed(1);
    this.setDegrees(this.currentPose.launchAngleDeg);
    SmartDashboard.putNumber("Degrees above horizontal", this.currentPose.launchAngleDeg);
  }

  public void decreaseLength()// Used for testing
  {
    if (this.currentPose.launchAngleDeg > C_ACTUATOR_MIN_DEG) {
      this.currentPose.launchAngleDeg -= 3;
    }
    // actuator.setPosition(actuatorPos);
    // actuator.setSpeed(-1);
    this.setDegrees(this.currentPose.launchAngleDeg);
    // actuator.setPosition(0);
    SmartDashboard.putNumber("Degrees above horizontal", this.currentPose.launchAngleDeg);
  }

  // Set degrees above horizontal.
  public void setDegrees(double degreesHorizontal){
    currentPose.setAngleDeg(degreesHorizontal);
    actuator.setPosition(currentPose.launchAngleExtension);
  }

  public void setShooterPose(ShooterPose shooterPose){
    this.currentPose = shooterPose;
    updateShooterPose();
  }

  public void updateShooterPose(){
    setDegrees(currentPose.launchAngleDeg);
    setShooterVelocityPID(currentPose.speedRPM);
  }

  public ShooterPose distanceToDegrees(double metersFromGoal){
    double approachAngleRadians = Math.toRadians(C_GOAL_APPROACH_DEGREES);
    
    // Difference in height between shooter and goal (y1)
    double heightDiff = C_SHOOT_HEIGHT_METERS - C_GOAL_HEIGHT_METERS;
    
    // X component of velocity (vx)
    double vx = Math.sqrt(C_GRAV_ACCEL*metersFromGoal/(2*(Math.tan(approachAngleRadians) - heightDiff/metersFromGoal)));
    
    // Y component of velocity when passing through goal (vy0)
    double vyGoal = vx*Math.tan(approachAngleRadians);

    // Velocity of ball as it passes through the goal (combined components) (v0)
    double vGoal = Math.sqrt(Math.pow(vx, 2) + Math.pow(vyGoal, 2));

    // Y component of velocity when launched (vy1)
    double vyLaunch = vyGoal - (C_GRAV_ACCEL*metersFromGoal)/vx;

    // Launch angle in radians (theta1)
    double launchAngleRadians = -Math.atan(vyLaunch/vx);
    SmartDashboard.putNumber("angle", Math.toDegrees(launchAngleRadians));
    
    double launchSpeedMPS = (Math.sqrt(Math.pow(vx, 2) + Math.pow(vyLaunch, 2))) * 2.3;
    // *2 
    return new ShooterPose(launchAngleRadians, launchSpeedMPS, false, false);
  }

  public ShooterPose getShooterPose(){
    return this.currentPose;
  }

  public class ShooterPose{
    public double launchAngleDeg;
    public double launchAngleRad;
    public double launchAngleExtension;
    
    public double speedRPM;
    public double speedMPS;

    public ShooterPose(double launchAngle, double speed, boolean isDeg, boolean isRPM){
      if(isDeg == true){
        setAngleDeg(launchAngle);
      }
      else if(isDeg == false){
        setAngleRad(launchAngle);
      }

      if(isRPM == true){
        setLaunchRPM(Math.max(speed, C_MIN_SPEED));
      }
      else if(isRPM == false){
        setLaunchMPS(speed);
      }
    }

    public ShooterPose(double launchAngleDeg, double speedRPM){
      this(launchAngleDeg, speedRPM, false, true);
    }

    public double degreesToExtension(double degreesHorizontal){
      // See https://docs.google.com/document/d/1j0m0NdNlVOw_fCRlhDM2ct6ic74NRDTYBwbQS5yPkpQ/edit?usp=sharing
      
      if (degreesHorizontal < C_ACTUATOR_MIN_DEG) {
        degreesHorizontal = C_ACTUATOR_MIN_DEG;
      }
      if (degreesHorizontal > C_ACTUATOR_MAX_DEG) {
        degreesHorizontal = C_ACTUATOR_MAX_DEG;
      }
      // Sets the angles within the correct bounds

      double degreesToB = 90 - degreesHorizontal; //                                                    Step 1
      degreesToB -= C_DEGREES_DIFFERENCE; //                                                            Step 2
      //SmartDashboard.putNumber("Degrees", degreesToB); // For testing
      double radiansToB = Math.toRadians(degreesToB); //                                                Conversion for Math.cos
      double totalC = Math.sqrt((Math.pow(C_CENTER_DISTANCE_CM, 2) + Math.pow(C_HOOD_RADIUS_CM, 2)) //  Step 3 using law of cosines
          - (2 * C_CENTER_DISTANCE_CM * C_HOOD_RADIUS_CM * Math.cos(radiansToB))); //                   to find c (actuator length)
      //SmartDashboard.putNumber("Total Actuator Length", totalC); // For testing
      double actuatorExtension = totalC - C_ACTUATOR_RETRACTED_CM; //                                   Step 4
      actuatorExtension /= C_ACTUATOR_FULL_EXTENSION_CM; //                                             Convert an extension range of
      //                                                                                                14 cm to 1.0 for the actuator
      //                                                                                                setPosition function
      SmartDashboard.putNumber("Actuator extension", actuatorExtension);// For testing
      
      return actuatorExtension;
    }

    public void setAngleDeg(double degrees){
      this.launchAngleDeg = degrees;
      this.launchAngleRad = Math.toRadians(degrees);
      this.launchAngleExtension = degreesToExtension(this.launchAngleDeg);
    }

    public void setAngleRad(double rad){
      this.launchAngleRad = rad;
      this.launchAngleDeg = Math.toDegrees(rad);
      this.launchAngleExtension = degreesToExtension(this.launchAngleDeg);
    }

    public void setLaunchRPM(double rpm){
      this.speedRPM = rpm;
      this.speedMPS = rpm * (Math.PI*C_FLYWHEEL_DIAMETER_METERS) / 60.0;
    }

    public void setLaunchMPS(double mps){
      this.speedMPS = mps;
      this.speedRPM = mps * 60.0 / (Math.PI*C_FLYWHEEL_DIAMETER_METERS);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("speed", this.getShooterVel());

    // This method will be called once per scheduler run
  }
}
