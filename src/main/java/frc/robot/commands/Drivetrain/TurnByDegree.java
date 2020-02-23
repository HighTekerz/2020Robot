/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnByDegree extends CommandBase {

  double speed, degreesToTurn, currentDegrees, targetDegrees;
  boolean clockwise;
  Drivetrain dt = Drivetrain.getInstance();

  /**
   * 
   */
  public TurnByDegree(double degreesToTurn, double speed, boolean clockwise) {
    addRequirements(Drivetrain.getInstance());
    this.speed = speed;
    this.degreesToTurn = degreesToTurn;
    this.clockwise = clockwise;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDegrees = dt.getAngle();
    if(clockwise){
      targetDegrees = currentDegrees - Math.abs(degreesToTurn);
      speed = Math.abs(speed);
    }
    else{
      targetDegrees = currentDegrees + Math.abs(degreesToTurn);
      speed = -Math.abs(speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.setWheelSpeed(-speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentDegrees = dt.getAngle();
    if(clockwise && currentDegrees <= targetDegrees ||
      !clockwise && currentDegrees >= targetDegrees){
        return true;
    }
    else {
      return false;
    }
  }
}