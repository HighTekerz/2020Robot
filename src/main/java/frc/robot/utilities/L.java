/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class L {
    public static void og(Object data){
        System.out.println(data.toString());
    }

    public static void ogSD(String title, Object data){
        if(data instanceof String){
            SmartDashboard.putString(title, (String)data);
        }
        else if(data instanceof Integer){
            SmartDashboard.putNumber(title, (int)data);
        }
        else if(data instanceof Double){
            SmartDashboard.putNumber(title, (double)data);
        }
        else if(data instanceof Sendable){
            SmartDashboard.putData(title, (Sendable)data);
        }
        else{
            System.out.println(title + " did not write to the SD");
        }
    }
}
