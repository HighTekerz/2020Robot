/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.L;
import frc.robot.MechanoidAtlas;

public class Shooter extends SubsystemBase {

	// Unknown Numbers
	private TalonFX leftFlywheel = new TalonFX(MechanoidAtlas.leftShooterMotor),
			rightFlywheel = new TalonFX(MechanoidAtlas.rightShooterMotor);

	public static final double POSITION1 = 30, 
											POSITION2 = 30, 
											POSITION3 = 30;

	public static Shooter shooter;

	private boolean spinning = false;

	public static Shooter getInstance() {
		if (shooter == null) {
			shooter = new Shooter();
		}
		return shooter;
	}

	
	private double p, i, d, fF = 0.0, loopLengthInSeconds = .02;

	public final PIDController pIDLoop = new PIDController(p, i, d, loopLengthInSeconds);

	/**
	 * Creates a new Shooter.
	 */
	private Shooter() {
		leftFlywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		pIDLoop.enableContinuousInput(-10000, 10000);
	}

	public void setSetpoint(double setpoint) {
		pIDLoop.setSetpoint(setpoint);
		spinning = true;
	}

	public void stopPID() {
		spinning = false;
		leftFlywheel.set(TalonFXControlMode.Disabled, 0);
	}

	public void runPID() {
		double pIDOutput = pIDLoop.calculate(leftFlywheel.getSelectedSensorVelocity());
		leftFlywheel.set(ControlMode.PercentOutput, pIDOutput + fF);
		L.ogSD("PID Output", pIDOutput);
		L.ogSD("LeftFly Velocity", leftFlywheel.getSelectedSensorVelocity());
	}

	public double getFF(){
		return fF;
	}

	public void setFF(double newValue){
		fF = newValue;
	}


	@Override
	public void periodic() {
		if (spinning){
			runPID();
		}	
	}
}