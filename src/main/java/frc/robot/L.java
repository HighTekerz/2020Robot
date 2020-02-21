/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class L {
    
    private static L l = null;

    public static L getInstance() {
		if(l == null){
			l = new L();
		}
		return l;
	}

    public void og(Object data){
        System.out.println(data.toString());
    }

    public void ogSD(String title, Object data){
        // SmartDashboard.putData(title, data);
    }
}
