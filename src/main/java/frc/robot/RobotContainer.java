/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Drivetrain.DriveForDistance;
import frc.robot.commands.Drivetrain.DriveWithJoy;
import frc.robot.commands.Drivetrain.TurnByDegree;
import frc.robot.commands.Shooter.PIDTuner;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Throat;
import frc.robot.subsystems.WheelLooker;
import frc.robot.subsystems.WheelSpinner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Drivetrain dt = Drivetrain.getInstance();

  //Meat is driver, Beat is Co-Pilot
  private Joystick meatStick = new Joystick(0);
  private Joystick beatStick = new Joystick(1);
  
  private static class joyButts{
    public static int A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4, LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
    BACK = 7, START = 8, RIGHT_THUMBSTICK_BUTTON = 9, LEFT_THUMBSTICK_BUTTON = 10, LEFT_TRIGGER = 2,
    RIGHT_TRIGGER = 3;
  }

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    dt.setDefaultCommand(new DriveWithJoy());
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  Shooter shooter = Shooter.getInstance();
  private void configureButtonBindings() {

    new JoystickButton(meatStick, joyButts.START).whenPressed(new InstantCommand(()-> shooter.setSetpoint(shooter.POSITION1)));

    // > > > >Example Button< < < <
    DPadButton go2Feet = new DPadButton(meatStick, 0, 0);
    go2Feet.whenPressed(new DriveForDistance(24.0, 0.3));

    DPadButton goBack2Feet = new DPadButton(meatStick, 0, 180);
    goBack2Feet.whenPressed(new DriveForDistance(-24.0, 0.3));

    DPadButton turn45DegreesLeft = new DPadButton(meatStick, 0, 270);
    turn45DegreesLeft.whenPressed(new TurnByDegree(45, 0.3, false));

    DPadButton turn45DegreesRight = new DPadButton(meatStick, 0, 90);
    turn45DegreesRight.whenPressed(new TurnByDegree(45, 0.3, true));

  }
  
  public double getDriverLeftStickX(){
    return meatStick.getRawAxis(0);
  }

  public double getDriverLeftStickY(){
    return meatStick.getRawAxis(1);
  }

  public double getDriverRightStickY(){
    return -meatStick.getRawAxis(5);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
