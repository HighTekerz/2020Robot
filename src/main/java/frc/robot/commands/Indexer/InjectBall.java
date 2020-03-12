/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Indexer;

import javax.swing.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class InjectBall extends CommandBase {

    Indexer indexer = Indexer.getInstance();

    Boolean hasABallPassed = false;

    /**
     * Creates a new InjectBall.
     */
    public InjectBall() {
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hasABallPassed = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexer.setSpeed(0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.setSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (hasABallPassed && indexer.getThirdSensor()) {
            return true;
        } else {
            if (indexer.getThirdSensor()) {
                return false;
            } else {
                hasABallPassed = true;
                return false;
            }
        }
    }
}
