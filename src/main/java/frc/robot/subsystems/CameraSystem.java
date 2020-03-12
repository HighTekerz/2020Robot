/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSystem extends SubsystemBase {

    public static CameraSystem camSys = null;

    public static CameraSystem getInstance(){
        if(camSys == null){
            return camSys = new CameraSystem();
        }
        else{
            return camSys;
        }
    }

    /**
     * Creates a new CameraSystem.
     */
    private CameraSystem() {
    }

    public void swapToDriverCam(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }

    public void swapToLimeCam(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }

    public void setLightOn(boolean turnOn){
        if(turnOn){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        }
        else{
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
