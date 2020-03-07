/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;

public class WheelSpinner extends SubsystemBase {

  private TalonSRX jeopardyMotor = new TalonSRX(MechanoidAtlas.jeopardyMotor);

  private static WheelSpinner wheelSpinner = null;

  public static WheelSpinner getInstance(){
    if(wheelSpinner == null){
      return wheelSpinner = new WheelSpinner();
    }
    else{
      return wheelSpinner;
    }
  }

  /**
   * Creates a new WheelSpinner.
   */
  public WheelSpinner() {
  }

  public void setSpeed(double speed){
    jeopardyMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
