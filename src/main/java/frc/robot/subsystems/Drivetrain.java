/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.L;
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

	TalonSRX hogBearer = new TalonSRX(20);
	PigeonIMU hogEra = new PigeonIMU(hogBearer);

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

	public double getEnc(){
		return leftDriveMotor1.getSelectedSensorPosition();
	}

	double[] yawPitchRollArray = new double[3];

	public double getAngle() {
	  hogEra.getYawPitchRoll(yawPitchRollArray);
	  return yawPitchRollArray[0];
	}
  
	public double getPitch(){
	  hogEra.getYawPitchRoll(yawPitchRollArray);
	  return yawPitchRollArray[1];
	}
  
	public double getRoll(){
		hogEra.getYawPitchRoll(yawPitchRollArray);
	  	return yawPitchRollArray[2];
	}

	@Override
	public void periodic() {
		L.getInstance().ogSD("Angle", getAngle());
		L.getInstance().ogSD("Roll", getRoll());
		L.getInstance().ogSD("Pitch", getPitch());
	}
}
