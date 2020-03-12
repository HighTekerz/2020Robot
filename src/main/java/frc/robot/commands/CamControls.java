/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CameraSystem;
import frc.robot.utilities.L;

public class CamControls extends CommandBase {
    CameraSystem camSys = CameraSystem.getInstance();

  /**
   * Creates a new CamControls.
   */
  public CamControls() {
    addRequirements(camSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    L.ogSD("Camera DriverMode",new InstantCommand(()-> camSys.swapToDriverCam()));
    L.ogSD("Camera LimeMode",new InstantCommand(()-> camSys.swapToLimeCam()));
    L.ogSD("Light On", new InstantCommand(()-> camSys.setLightOn(true)));
    L.ogSD("Light Off", new InstantCommand(()-> camSys.setLightOn(false)));
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
