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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.L;
import frc.robot.MechanoidAtlas;

public class Drivetrain extends SubsystemBase {
	/**
	 * Creates a new Drivetrain.
	 */
	private static Drivetrain drivetrain = null;
	
	// TalonFX leftDriveMotor1 = new TalonFX(4),
	//  			 leftDriveMotor2 = new TalonFX(1),
	//  			 rightDriveMotor1 = new TalonFX(2),
	//   			 rightDriveMotor2 = new TalonFX(3);

	CANSparkMax leftDriveMotor1 = new CANSparkMax(1, MotorType.kBrushless),
	                        leftDriveMotor2 = new CANSparkMax(3, MotorType.kBrushless),
	 						rightDriveMotor1 = new CANSparkMax(2, MotorType.kBrushless),
							 rightDriveMotor2 = new CANSparkMax(4, MotorType.kBrushless);
	CANEncoder leftEnc = leftDriveMotor1.getEncoder();

	TalonSRX hogBearer = new TalonSRX(10);
	PigeonIMU hogEra = new PigeonIMU(hogBearer);

	// Talons
	// public static final double TICKS_PER_INCH = 2048/18.5;
	
	// Sparks
	public static final double REVS_PER_INCH = 10.71/18.85;

	public static Drivetrain getInstance() {
		if(drivetrain == null){
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}

	private Drivetrain() {
		// leftDriveMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	}

	public void driveTank(double leftSpeed, double rightSpeed){
		// leftDriveMotor1.set(ControlMode.PercentOutput, leftSpeed);
		// leftDriveMotor2.set(ControlMode.PercentOutput, leftSpeed);
		// rightDriveMotor1.set(ControlMode.PercentOutput, rightSpeed);
		// rightDriveMotor2.set(ControlMode.PercentOutput, rightSpeed);

		// Negative numbers make right go forward
		rightDriveMotor1.set(rightSpeed);
		rightDriveMotor2.set(rightSpeed);
		leftDriveMotor1.set(leftSpeed);
		leftDriveMotor2.set(leftSpeed);
	}

	double leftSpeed;
	double rightSpeed;
	public void driveArcade(double throttle, double turn){
		leftSpeed = throttle + turn;
		rightSpeed = throttle - turn;
		L.ogSD("Turn", turn);
		L.ogSD("Throttle", throttle);

		driveTank(leftSpeed, -rightSpeed);
	}

	public void driveModified(double throttle, double turn){
		if (throttle < 0.075 && throttle > -0.075){
			throttle = 0;
		}
		if (turn < 0.075 && turn > -0.075){
			turn = 0;
		}

		leftSpeed = Math.pow((throttle + turn), 3);
		rightSpeed = Math.pow((throttle - turn), 3);

		driveTank(leftSpeed, -rightSpeed);
	}

	public double getEnc(){
		return leftEnc.getPosition();
		// return leftDriveMotor2.getSelectedSensorPosition(0);
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
