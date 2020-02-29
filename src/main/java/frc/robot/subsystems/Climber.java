/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;

public class Climber extends SubsystemBase {
  
  DoubleSolenoid climberPistons = new DoubleSolenoid(MechanoidAtlas.cliSol1, MechanoidAtlas.cliSol2);
  DigitalInput magnetSensor = new DigitalInput(MechanoidAtlas.climberMagnetSensor);

  public static Climber climber = null;

  public static Climber getInstance(){
    if(climber == null){
      climber = new Climber();
    }
    return climber;
  }

  private Climber() {
  }

  public void climbUp(){
    climberPistons.set(Value.kForward);
  }

  public void climbDown(){
    climberPistons.set(Value.kReverse);
  }

  public void climbStop(){
    climberPistons.set(Value.kOff);
    //                        ^ koff koff XD 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}