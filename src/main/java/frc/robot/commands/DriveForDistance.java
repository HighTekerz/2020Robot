/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance extends CommandBase {

  private double ticksToTravel, speed, currentInches, tickTarget;

  /**
   * Drive autonomously for a number of inches
   * 
   * @param ticksToTravel inches to travel
   * @param speed speed to travel at
   */
  public DriveForDistance(double ticksToTravel, double speed) {
    addRequirements(Drivetrain.getInstance());
    this.ticksToTravel = ticksToTravel;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentInches = Drivetrain.getInstance().getEnc();
    tickTarget = currentInches + ticksToTravel;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drivetrain.getInstance().setWheelSpeed(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentInches = Drivetrain.getInstance().getEnc();
    // if(){
      return false;
    // }
  }
}
