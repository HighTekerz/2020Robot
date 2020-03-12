/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;

public class Intake extends SubsystemBase {

  Solenoid intakeSolenoid =  new Solenoid(MechanoidAtlas.intakeSolenoid);

  TalonSRX intakeMotor = new TalonSRX(MechanoidAtlas.intakeMotor);

  private static Intake intake = null;

  public static Intake getInstance(){
    if(intake == null){
      return intake = new Intake();
    }
    else{
      return intake;
    }
  }

  /**
   * Creates a new Intake.
   */
  private Intake() {

  }

  public void armsDown(){
    intakeSolenoid.set(true);
  }

  public void armsUp(){
    intakeSolenoid.set(false);
  }

  public void setSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
