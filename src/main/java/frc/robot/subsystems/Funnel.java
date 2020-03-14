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

public class Funnel extends SubsystemBase {

  TalonSRX funnelMotor = new TalonSRX(MechanoidAtlas.funnelMotor);
   
   public boolean shouldBeRunning = false;

   private static Funnel funnel = null;

  public static Funnel getInstance(){
    if(funnel == null){
      return funnel =  new Funnel();
    }
    else{
      return funnel;
    }
  }

  /**
   * Creates a new Funnel.
   */
  public Funnel() {
    // setDefaultCommand(new AutoSuck());
  }

  public void setSpeed(double speed){
    funnelMotor.set(ControlMode.PercentOutput, speed);
  }
 
  public void setShouldBeRunning(boolean shouldBeRunning) {
    this.shouldBeRunning = shouldBeRunning;
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}