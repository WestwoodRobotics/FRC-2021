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
import frc.robot.Constants.ShooterConstants.E_SHOOTER_POS;

public class Shooter extends SubsystemBase {
  // variables
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;

  private Servo actuator;

  private double speedSetpoint = 0.0;
  private E_SHOOTER_POS pos;

  private double actuatorPos = 0.0;// Position 0.0 to 1.0

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

    pos = E_SHOOTER_POS.CLOSE;

    // Actuator stuff

    actuator = new Servo(P_ACTUATOR);

    // actuator.setBounds(C_ACTUATOR_MAX_PWM_MS, 1800,
    // ((C_ACTUATOR_MIN_PWM_MS+C_ACTUATOR_MAX_PWM_MS)/2), 1200,
    // C_ACTUATOR_MIN_PWM_MS);
    actuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  public void stopShooter() {
    shooterMotor1.stopMotor();
  }

  public void setShooterPercentage(double percent) {
    shooterMotor1.set(percent);
  }

  public void setShooterVoltage(double voltage) {
    shooterMotor1.setVoltage(voltage);
  }

  public void setShooterVelocityPID(double rpm) {
    this.speedSetpoint = rpm;
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

  public boolean isFlyWheelReady() {
    return (Math.abs(this.getShooterVel() - this.speedSetpoint) < C_SHOOTER_SPEED_TOLERANCE);
  }

  // Actuator

  public void setLength(double length)// Takes in value 0.0 to 1.0
  {
    actuator.setPosition(length);
  }

  public void increaseLength()// Used for testing
  {
    if (actuatorPos < C_ACTUATOR_MAX_DEG) {
      actuatorPos += 3;
    }
    // actuator.setPosition(actuatorPos);
    // actuator.setSpeed(1);
    this.setDegrees(actuatorPos);
    SmartDashboard.putNumber("Degrees above horizontal", actuatorPos);
  }

  public void decreaseLength()// Also used for testing
  {
    if (actuatorPos > C_ACTUATOR_MIN_DEG) {
      actuatorPos -= 3;
    }
    // actuator.setPosition(actuatorPos);
    // actuator.setSpeed(-1);
    this.setDegrees(actuatorPos);
    // actuator.setPosition(0);
    SmartDashboard.putNumber("Degrees above horizontal", actuatorPos);
  }

  // Set degrees above horizontal.
  // https://docs.google.com/document/d/1j0m0NdNlVOw_fCRlhDM2ct6ic74NRDTYBwbQS5yPkpQ/edit?usp=sharing
  public void setDegrees(double degreesHorizontal){
    if (degreesHorizontal < C_ACTUATOR_MIN_DEG) {
      degreesHorizontal = C_ACTUATOR_MIN_DEG;
    }
    if (degreesHorizontal > C_ACTUATOR_MAX_DEG) {
      degreesHorizontal = C_ACTUATOR_MAX_DEG;
    }
    double degrees = 90 - degreesHorizontal;// Step 1
    degrees -= C_DEGREES_DIFFERENCE;// Step 2 to 3
    SmartDashboard.putNumber("Degrees", degrees);
    double radians = Math.toRadians(degrees);
    double totalRadius = Math.sqrt((Math.pow(C_CENTER_DISTANCE_CM, 2) + Math.pow(C_HOOD_RADIUS_CM, 2))
        - (2 * C_CENTER_DISTANCE_CM * C_HOOD_RADIUS_CM * Math.cos(radians)));// Step 6
    SmartDashboard.putNumber("Total Radius", totalRadius);
    double actuatorExtension = totalRadius - C_ACTUATOR_RETRACTED_CM;
    actuatorExtension /= C_ACTUATOR_EXTENSION_CM;
    SmartDashboard.putNumber("Actuator extension", actuatorExtension);
    actuator.setPosition(actuatorExtension);
  }

  public void setShooterPose(ShooterPose shooterPose){
    setDegrees(shooterPose.launchAngleDeg);
    setShooterVelocityPID(shooterPose.speedRPM);
  }

  public ShooterPose distanceToDegrees(double metersFromGoal){
    double approachAngleRadians = Math.toRadians(C_GOAL_APPROACH_DEGREES);
    
    // Difference in height between shooter and goal (y1)
    double heightDiff = C_SHOOT_HEIGHT_METERS - C_GOAL_HEIGHT_METERS;
    
    // X component of velocity (vx)
    double vx = Math.sqrt(C_GRAV_ACCEL*metersFromGoal/(2*(Math.tan(approachAngleRadians) - heightDiff)));
    
    // Y component of velocity when passing through goal (vy0)
    double vyGoal = vx*Math.tan(approachAngleRadians);

    // Velocity of ball as it passes through the goal (combined components) (v0)
    double vGoal = Math.sqrt(Math.pow(vx, 2) + Math.pow(vyGoal, 2));

    // Y component of velocity when launched (vy1)
    double vyLaunch = vyGoal - (C_GRAV_ACCEL*metersFromGoal)/vx;

    // Launch angle in radians (theta1)
    double launchAngleRadians = -Math.atan(vyLaunch/vx);
    
    double launchSpeedMPS = Math.sqrt(Math.pow(vx, 2) + Math.pow(vyLaunch, 2));

    return new ShooterPose(launchAngleRadians, launchSpeedMPS, false, false);
  }

  private class ShooterPose{
    public double launchAngleDeg;
    public double launchAngleRad;
    public double speedRPM;
    public double speedMPS;

    public ShooterPose(double launchAngle, double speed, boolean isDeg, boolean isRPM){
      if(isDeg == true){
        this.launchAngleDeg = launchAngle;
        this.launchAngleRad = Math.toRadians(launchAngle);
      }
      else if(isDeg == false){
        this.launchAngleRad = launchAngle;
        this.launchAngleDeg = Math.toDegrees(launchAngle);
      }

      if(isRPM == true){
        this.speedRPM = speed;
        this.speedMPS = speed * C_FLYWHEEL_DIAMETER_METERS / 60.0;
      }
      else if(isRPM == false){
        this.speedMPS = speed;
        this.speedRPM = speed * 60.0 / C_FLYWHEEL_DIAMETER_METERS;
      }
    }

    public ShooterPose(double launchAngleDeg, double speedRPM){
      this(launchAngleDeg, speedRPM, false, true);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("speed", this.getShooterVel());

    // This method will be called once per scheduler run
  }
}
