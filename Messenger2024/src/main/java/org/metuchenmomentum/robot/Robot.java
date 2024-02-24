// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.metuchenmomentum.robot;

import org.metuchenmomentum.robot.subsystems.intake.IntakeSimulation;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    private final IntakeSimulation intakeSim = new IntakeSimulation();
    private final XboxController controller = new XboxController(0);

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void simulationPeriodic() {
        intakeSim.simulationPeriodic();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        if (controller.getYButtonPressed()) {
            intakeSim.reachSetpoint();
        } else {
            intakeSim.stop();
        }
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
        
    }
}
