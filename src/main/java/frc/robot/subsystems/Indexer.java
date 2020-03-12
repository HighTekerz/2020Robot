/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MechanoidAtlas;
import frc.robot.utilities.L;

public class Indexer extends SubsystemBase {

    TalonSRX elevatorMotor = new TalonSRX(MechanoidAtlas.elevatorMotor);

    DigitalInput ballSensor1 = new DigitalInput(MechanoidAtlas.indexBeam1),
            ballSensor2 = new DigitalInput(MechanoidAtlas.indexBeam2),
            ballSensor3 = new DigitalInput(MechanoidAtlas.indexBeam3);

    private static Indexer elevIndexer = null;

    public static Indexer getInstance() {
        if (elevIndexer == null) {
            return elevIndexer = new Indexer();
        } else {
            return elevIndexer;
        }
    }

    /**
     * Creates a new ElevIndexer.
     */
    private Indexer() {

    }

    public void setSpeed(double speed) {
        elevatorMotor.set(ControlMode.PercentOutput, speed);
    }

    public boolean getThirdSensor() {
        return ballSensor3.get();
    }

    public boolean getFirstSensor() {
        return ballSensor1.get();
    }

    public boolean getSecondSensor() {
        return ballSensor2.get();
    }

    @Override
    public void periodic() {
        L.ogSD("Sensor1", getFirstSensor());
    }
}
