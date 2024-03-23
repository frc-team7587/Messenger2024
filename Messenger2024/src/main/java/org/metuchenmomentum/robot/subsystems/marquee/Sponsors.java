package org.metuchenmomentum.robot.subsystems.marquee;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sponsors extends SubsystemBase {
    public static final List<DisplayMessage> sponsorList;
    private String allianceColor = "";
    private int acR = 0;
    private int acG = 0;
    private int acB = 0;

    public Sponsors(String alliance) {
        this.allianceColor = alliance;
        if (alliance.contains("Blue")) {
            this.acB = 250;
            allianceColor = "Go Blue!!!";
        }
        if (alliance.contains("Red")) {
            this.acR = 250;
            allianceColor = "Go Red!!!";
        }
        //custom message
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText(allianceColor)
                    .setForegroundRed(acR)
                    .setForegroundGreen(acG)
                    .setForegroundBlue(acB)
                    .setDelay1(40));
    }

    //constructor for static initialization, creates each sponsor message
    static {
        sponsorList = new ArrayList<>();
        //Thank you msg
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Thank you to our Sponsors:")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40)
        );
        //sponsor 1
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("First Robotics Competition <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(0)
                    .setForegroundBlue(0)
                    .setBackgroundRed(0)
                    .setBackgroundGreen(0)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //sponsor 2
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Picatinny STEM")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40));
        //sponsor 3
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("J*Star Research Inc.")
                    .setForegroundRed(72)
                    .setForegroundGreen(61)
                    .setForegroundBlue(139)
                    .setDelay1(40));
        //sponsor 4
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("ExxonMobil")
                    .setForegroundRed(230)
                    .setForegroundGreen(0)
                    .setForegroundBlue(0)
                    .setDelay1(40));
        //sponsor 5
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("NOKIA Bell Labs")
                    .setForegroundRed(25)
                    .setForegroundGreen(25)
                    .setForegroundBlue(220)
                    .setDelay1(40));
        //sponsor 6
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("ADP")
                    .setForegroundRed(250)
                    .setForegroundGreen(10)
                    .setForegroundBlue(10)
                    .setDelay1(40));
        //sponsor 7
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("B.P.O.E")
                    .setForegroundRed(10)
                    .setForegroundGreen(60)
                    .setForegroundBlue(250)
                    .setDelay1(40));
        //sponsor 8
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Cole Solutions LLC")
                    .setForegroundRed(135)
                    .setForegroundGreen(206)
                    .setForegroundBlue(250)
                    .setDelay1(40));
        //sponsor 8.2
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Excellence Through Collaboration")
                    .setForegroundRed(135)
                    .setForegroundGreen(206)
                    .setForegroundBlue(250)
                    .setDelay1(40));
        //special thanks
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("And special thanks to our mentors: ")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //mentor 1
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Mr.Smedley <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //mentor 2
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Mrs.McGrory <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //mentor 3
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Mr.McGrory <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //mentor 4
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Mr.Mintz <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //mentor 5
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Mr.Tang <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
        //mentor 6
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Mr.Fuentes <3")
                    .setForegroundRed(255)
                    .setForegroundGreen(30)
                    .setForegroundBlue(0)
                    .setBackgroundRed(1)
                    .setBackgroundGreen(1)
                    .setBackgroundBlue(1)
                    .setDelay1(40));
    }

    public List<DisplayMessage> getList(){
        return sponsorList;
    }
}
