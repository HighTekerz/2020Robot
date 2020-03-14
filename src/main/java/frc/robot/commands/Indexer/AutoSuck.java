/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Funnel;

public class AutoSuck extends CommandBase {
    Funnel funnel = Funnel.getInstance();
    Indexer indexer = Indexer.getInstance();

    Timer timer = new Timer();

    /**
     * Creates a new AutoSuck.
     */
    public AutoSuck() {
        addRequirements(indexer);
        addRequirements(funnel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    boolean firstSensorSawBall;

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (indexer.getFirstSensor() || indexer.getSecondSensor()) {
            if (!firstSensorSawBall && indexer.getFirstSensor()) {
                firstSensorSawBall = true;
                timer.reset();
                funnel.setSpeed(0.2);
            }
            if (firstSensorSawBall && timer.get() > 0.25) {
                funnel.setSpeed(0.0);
                firstSensorSawBall = false;
            }
            indexer.setSpeed(0.2);
        } else {
            if (funnel.shouldBeRunning) {
                funnel.setSpeed(0.2);
            } else {
                funnel.setSpeed(0.0);
            }
            indexer.setSpeed(0.0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.setSpeed(0.0);
        funnel.setSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}