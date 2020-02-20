/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveWithJoy;

public class Drivetrain extends SubsystemBase {
	/**
	 * Creates a new Drivetrain.
	 */
	public static Drivetrain drivetrain = null;
	
	TalonFX leftDriveMotor1 = new TalonFX(0);
	TalonFX leftDriveMotor2 = new TalonFX(1);
	TalonFX rightDriveMotor1 = new TalonFX(2);
	TalonFX rightDriveMotor2 = new TalonFX(3);

	public static Drivetrain getInstance() {
		if(drivetrain == null){
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}

	private Drivetrain() {
		setDefaultCommand(new DriveWithJoy());
	}

	public void setWheelSpeed(double leftSpeed, double rightSpeed){
		leftDriveMotor1.set(ControlMode.PercentOutput, leftSpeed);
		leftDriveMotor2.set(ControlMode.PercentOutput, leftSpeed);
		rightDriveMotor1.set(ControlMode.PercentOutput, rightSpeed);
		rightDriveMotor2.set(ControlMode.PercentOutput, rightSpeed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
