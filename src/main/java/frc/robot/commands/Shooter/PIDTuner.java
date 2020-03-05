/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.L;
import frc.robot.subsystems.Shooter;

public class PIDTuner extends CommandBase {

  private Shooter shooter = Shooter.getInstance();

  private double p, i, d, setpoint;
  /**
   * Creates a new PIDTuner.
   */
  public PIDTuner() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    L.ogSD("PID [P]", 0);
    L.ogSD("PID [I]", 0);
    L.ogSD("PID [D]", 0);
    L.ogSD("PID [F]", 0);
    L.ogSD("PID [SP]", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setpoint = SmartDashboard.getNumber("PID [SP]", 0); 

  //  if(setpoint == 0){
    // shooter.stopPID();
    p = SmartDashboard.getNumber("PID [P]", 0);
    i = SmartDashboard.getNumber("PID [I]", 0);
    d = SmartDashboard.getNumber("PID [D]", 0);
     if(p != shooter.pIDLoop.getP() || i != shooter.pIDLoop.getI() || d != shooter.pIDLoop.getD()){
      shooter.pIDLoop.setPID(p, i, d); 
     }
  //  }
  //  else{
  //    if(setpoint != shooter.pIDLoop.getSetpoint())
    shooter.setSetpoint(setpoint);
  //  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}