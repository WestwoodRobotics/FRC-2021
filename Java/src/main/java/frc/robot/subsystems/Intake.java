// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.IntakeConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final WPI_TalonSRX intakeMotor1 = new WPI_TalonSRX(P_INTAKE_talSRX_1);
  private Solenoid intakeSol1 = new Solenoid(P_INTAKE_sol_1);

  public Intake() {
    intakeSol1.set(true);
    intakeMotor1.setInverted(true);
  }
  
  public void togglePiston(){
    intakeSol1.toggle();
  }

  public void pushIntake(){
    intakeSol1.set(true);
  }

  public void pullIntake(){
    intakeSol1.set(false);
  }

  public void intakeStop(){
    intakeMotor1.stopMotor();
  }

  public void intakeIn(){
    intakeMotor1.set(C_INTAKE_SPEED);
  }

  public void intakeOut(){
    intakeMotor1.set(-C_INTAKE_SPEED);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
