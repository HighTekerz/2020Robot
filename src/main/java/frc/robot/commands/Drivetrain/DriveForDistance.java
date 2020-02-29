/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.L;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance extends CommandBase {

  private double speed,
                         ticksToTravel, currentTicks, tickTarget;
  private boolean foreward;
  private Drivetrain drivetrain = Drivetrain.getInstance();

  /**
   * Drive autonomously for a number of ticks
   * 
   * @param ticksToTravel Ticks to travel
   * @param speed speed to travel at
   */
  public DriveForDistance(int ticksToTravel, double speed) {
    addRequirements(drivetrain);
    this.ticksToTravel = ticksToTravel;
    this.speed = speed;
  }

  public DriveForDistance(double inchesToTravel, double speed){
    addRequirements(drivetrain);
    this.ticksToTravel = inchesToTravel * Drivetrain.REVS_PER_INCH;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTicks = drivetrain.getEnc();
    if (speed <= 0 || ticksToTravel <= 0) {
      foreward = false;
      speed = -Math.abs(speed);
      tickTarget = currentTicks - Math.abs(ticksToTravel);
    } else {
      foreward = true;
      speed = Math.abs(speed);
      tickTarget = currentTicks + Math.abs(ticksToTravel);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveArcade(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentTicks = drivetrain.getEnc();
    if(foreward && currentTicks > tickTarget
    || !foreward && currentTicks < tickTarget){
      return true;
    }
    else{
      return false;
    }
  }
}