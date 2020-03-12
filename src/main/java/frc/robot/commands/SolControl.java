/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberStop;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.L;

public class SolControl extends CommandBase {
    Climber climber = Climber.getInstance();
  /**
   * Creates a new SolControl.
   */
  public SolControl() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      L.ogSD("Sol Forward", new ClimberUp());
      L.ogSD("Sol Off", new ClimberStop());
      L.ogSD("Sol Down", new ClimberDown());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
