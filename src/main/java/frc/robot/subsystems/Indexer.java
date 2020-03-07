/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;
import frc.robot.commands.Indexer.AutoSuck;

public class Indexer extends SubsystemBase {

  TalonSRX indexMotor = new TalonSRX(MechanoidAtlas.indexMotor);

  DigitalInput ballSensor1 = new DigitalInput(MechanoidAtlas.throatSwitch1),
   ballSensor2 = new DigitalInput(MechanoidAtlas.throatSwitch2),
   ballSensor3 = new DigitalInput(MechanoidAtlas.throatSwitch3);

  private static Indexer throat = null;

  public static Indexer getInstance(){
    if(throat == null){
      return throat =  new Indexer();
    }
    else{
      return throat;
    }
  }

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    setDefaultCommand(new AutoSuck());
  }

  public boolean getFirstSensor(){
    return ballSensor1.get();
  }

  public boolean getSecondSensor(){
    return ballSensor2.get();
  }

  public boolean getThirdSensor(){
    return ballSensor3.get();
  }

  public void setIndexSpeed(double speed){
    indexMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
