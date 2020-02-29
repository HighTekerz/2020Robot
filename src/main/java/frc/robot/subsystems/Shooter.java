/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;

public class Shooter extends SubsystemBase {
  
  // Unknown Numbers
  private TalonFX leftFlywheel = new TalonFX(MechanoidAtlas.leftShooterMotor);
  private TalonFX rightFlywheel = new TalonFX(MechanoidAtlas.rightShooterMotor);

  public static Shooter shooter;

  public static Shooter getInstance(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }

  /**
   * Creates a new Shooter.
   */
  private Shooter() {
    rightFlywheel.follow(leftFlywheel);
  }

  public void setSetpoint(double setpoint){
    
  }

  public void startPid(){

  }

  public void stopPid(){
    leftFlywheel.set(TalonFXControlMode.Disabled, 0);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
