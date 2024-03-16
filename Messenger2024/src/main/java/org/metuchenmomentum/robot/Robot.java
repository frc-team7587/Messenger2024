package org.metuchenmomentum.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.metuchenmomentum.robot.subsystems.marquee.*;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    //marquee
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private Marquee displayDriver;
    private DigitalOutput digOut0;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        /*/marquee
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto); 
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        digOut0 = new DigitalOutput(0);
        digOut0.setPWMRate(1000);
        digOut0.enablePWM(0.5);
        displayDriver = new Marquee(DisplayConnectionFactory.usbConnection());
*/
    }

    @Override
    public void robotPeriodic() {
        
        CommandScheduler.getInstance().run();
         //marquee
        //if (displayDriver != null) {
          //  displayDriver.robotPeriodic();
        //}
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
        //marquee
        //m_autoSelected = m_chooser.getSelected();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        /*/marquee
        switch (m_autoSelected) {
            case kCustomAuto:
                DisplayMessage smoothCrawl = new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("auto > driver")
                    .setForegroundRed(64)
                    .setForegroundGreen(32)
                    .setForegroundBlue(64)
                    .setDelay1(40);
                //displayConnection.send(smoothCrawl);
                break;
            case kDefaultAuto:
            default:
                DisplayMessage smoothCrawl2 = new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("auto > driver")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40);
                //displayConnection.send(smoothCrawl);
                break;
            //255,69,0
        }*/
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
