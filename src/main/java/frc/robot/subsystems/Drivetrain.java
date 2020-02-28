/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.L;
import frc.robot.MechanoidAtlas;

public class Drivetrain extends SubsystemBase {
	/**
	 * Creates a new Drivetrain.
	 */
	public static Drivetrain drivetrain = null;
	
	TalonFX leftDriveMotor1 = new TalonFX(MechanoidAtlas.leftDriveMotor1);
	TalonFX leftDriveMotor2 = new TalonFX(MechanoidAtlas.leftDriveMotor2);
	TalonFX rightDriveMotor1 = new TalonFX(MechanoidAtlas.rightDriveMotor1);
	TalonFX rightDriveMotor2 = new TalonFX(MechanoidAtlas.rightDriveMotor2);

	TalonSRX hogBearer = new TalonSRX(MechanoidAtlas.throatMotor);
	PigeonIMU hogEra = new PigeonIMU(hogBearer);

	public static Drivetrain getInstance() {
		if(drivetrain == null){
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}

	private Drivetrain() {
		leftDriveMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	}

	public void setWheelSpeed(double leftSpeed, double rightSpeed){
		// leftDriveMotor1.set(ControlMode.PercentOutput, leftSpeed);
		leftDriveMotor2.set(ControlMode.PercentOutput, leftSpeed);
		// rightDriveMotor1.set(ControlMode.PercentOutput, rightSpeed);
		// rightDriveMotor2.set(ControlMode.PercentOutput, rightSpeed);
	}

	public double getEnc(){
		return leftDriveMotor2.getSelectedSensorPosition(0);
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
		L.ogSD("Encoder", getEnc());
		L.ogSD("Angle", getAngle());
		L.ogSD("Roll", getRoll());
		L.ogSD("Pitch", getPitch());
	}
}
