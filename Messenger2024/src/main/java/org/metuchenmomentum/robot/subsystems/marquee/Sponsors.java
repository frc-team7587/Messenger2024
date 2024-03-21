package org.metuchenmomentum.robot.subsystems.marquee;

import java.util.ArrayList;
import java.util.List;

public class Sponsors {
    public static final List<DisplayMessage> sponsorList;

    //constructor for static initialization, creates each sponsor message
    static {
        sponsorList = new ArrayList<>();
        //sponsor 1
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Sponsor 1")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40));
        //sponsor 2
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Sponsor 2")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40));
        //sponsor 3
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Sponsor 3")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40));
        //custom message
        sponsorList.add(new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("Custom Message")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40));
    }

    private Sponsors() {
        
    }
}
