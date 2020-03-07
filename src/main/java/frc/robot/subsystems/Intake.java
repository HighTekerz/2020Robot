/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
