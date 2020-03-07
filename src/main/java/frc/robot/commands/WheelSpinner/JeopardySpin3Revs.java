/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.WheelSpinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelLooker;
import frc.robot.subsystems.WheelSpinner;

public class JeopardySpin3Revs extends CommandBase {

  WheelSpinner wS = WheelSpinner.getInstance();
  WheelLooker wL = WheelLooker.getInstance();

  /**
   * Creates a new JeopardySpin3Revs.
   */
  public JeopardySpin3Revs() {
    addRequirements(wL);
    addRequirements(wS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wL.startCounting();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wS.setSpeed(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wL.stopCounting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(wL.counted3Spins()){
      return true;
    }
    else{
      return false;
    }
  }
}
