package org.metuchenmomentum.robot.subsystems.marquee;

import java.util.Optional;

import org.metuchenmomentum.robot.subsystems.marquee.DisplayCommand;
import org.metuchenmomentum.robot.subsystems.marquee.DisplayConnection;
import org.metuchenmomentum.robot.subsystems.marquee.DisplayMessage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Test driver for the Metuchen Momentum marquee. The driver cycles
 * the display through test messages, displaying each message for
 * 6000 ms.
 */
public class Marquee {

    private static final String teamNumber = "7587";

    private final DisplayConnection displayConnection;

    private int callCount;
    private int callCount1;
    private int cycleCount;
  
    public Marquee(DisplayConnection displayConnection) {
        this.displayConnection = displayConnection;
        callCount = 0;
        cycleCount = 0;
        callCount1 = 0;
    }

    public void init() {
        rainbowPattern();
    }
    
    /**
     * The robot's {@code robotPeriodic} method should invoke this every cycle,
     * that is, every 20 milliseconds.
     */
    public void robotPeriodic() {
        /*callCount %= 300;
        if (callCount == 0) {
          switch (cycleCount) {
            case 0:
              blueTeamNumber();
              break;
            case 1:
              redTeamNumber();
              break;
          }
    
          cycleCount = (++cycleCount) % 18;
        }
        ++callCount;
        */
        Optional<Alliance> ally = DriverStation.getAlliance();
          if (ally.get() == Alliance.Red){
           redTextCrawl();
          }
          if (ally.get() == Alliance.Blue) {
            blueTextCrawl();
          }
        
    }
    public void autonomousInit() {
      DisplayMessage smoothCrawl = new DisplayMessage()
                    .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
                    .setText("auto > driver")
                    .setForegroundRed(255)
                    .setForegroundGreen(69)
                    .setForegroundBlue(0)
                    .setDelay1(40);
                
    }

  private void blueTextCrawl(){
     DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("GO BLUE!!! :)")
        .setForegroundRed(0)
        .setForegroundGreen(0)
        .setForegroundBlue(150)
        .setDelay1(40);
      displayConnection.send(smoothCrawl);
  }
  private void redTextCrawl(){
     DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("GO RED!!! :)")
        .setForegroundRed(255)
        .setForegroundGreen(0)
        .setForegroundBlue(0)
        .setDelay1(40);
      displayConnection.send(smoothCrawl);
  }
  private void blueTeamNumber() {
        // Fill the display with blue.
        DisplayMessage setBlue = new DisplayMessage()
        .setText(teamNumber)
        .setForegroundBlue(127)
        .setDisplayCommand(DisplayCommand.STATIC_TEXT);
      int bytesSent = displayConnection.send(setBlue);
      System.out.println(
        "Blue team number command >>>" + setBlue.toString() + "<<<");
      System.out.println(
        "Blue team number message sent to display, length = " + bytesSent + " bytes");
  }
  private void redTeamNumber() {
    // Fill the display with blue.
    DisplayMessage setBlue = new DisplayMessage()
      .setText(teamNumber)
      .setForegroundRed(127)
      .setDisplayCommand(DisplayCommand.STATIC_TEXT);
    int bytesSent = displayConnection.send(setBlue);
    System.out.println(
      "Red team number message sent to display, length = " + bytesSent + " bytes");
  }

  private void fillWithGreen() {
    DisplayMessage greenFill = new DisplayMessage()
        .setForegroundGreen(63)
        .setDisplayCommand(DisplayCommand.FILL_WITH_COLOR);
    int bytesSent = displayConnection.send(greenFill);
    System.out.println(
      "Green fill pattern command sent to display, length = " + bytesSent);
  }
  private void errorPattern() {
    DisplayMessage displayError = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.ERROR)
        .setDelay1(100);
    int bytesSent = displayConnection.send(displayError);
    System.out.println(
      "Display error pattern command set to display, length = " + bytesSent);
  }

  private void rainbowPattern() {
    DisplayMessage displayRainbow = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.RIPPLING_RAINBOW)
        .setDelay1(100);
    displayConnection.send(displayRainbow);
}

  private void scrollText() {
    DisplayMessage textCrawl = new DisplayMessage()
        .setText("Text crawl ...")
        .setDisplayCommand(DisplayCommand.SCROLLING_TEXT)
        .setForegroundGreen(32)
        .setForegroundBlue(32)
        .setDelay1(50)
        .setDelay2(50);
    int bytesSent = displayConnection.send(textCrawl);
    System.out.println(
      "Text crawl command sent to display, length = " + bytesSent);
  }

  private void singlePixelNaturalOrder() {
    DisplayMessage naturalOrder = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.SINGLE_PIXEL_NATURAL_ORDER)
        .setForegroundRed(32)
        .setForegroundGreen(32)
        .setDelay1(50);
    displayConnection.send(naturalOrder);
  }

  private void singlePixelLeftToRight() {
    DisplayMessage leftToRight = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.SINGLE_PIXEL_LEFT_TO_RIGHT)
        .setForegroundRed(32)
        .setForegroundBlue(32)
        .setDelay1(50);
    displayConnection.send(leftToRight);
  }

  private void smoothTextCrawl() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("---Smooth---")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void announceColorErrorHandlingTest() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("RGB Error Tests Follows")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void overSizedColorTest() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Over Size Color")
        .setForegroundRed(999)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void negativeColorTest() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Over Size Color")
        .setForegroundRed(-999)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void announceDelay1Test() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Delay 1 Test Follows")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void negativeDelay1Test() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Negative delay 1 Test")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(-1);
    displayConnection.send(smoothCrawl);
  }

  private void announceDelay2Test() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Delay 2 Test Follows")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void negativeDelay2Test() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Negative delay 2 Test")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40)
        .setDelay2(-1);
    displayConnection.send(smoothCrawl);
  }

  private void announceOversizeMessageTest() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("Oversize Message Test Follows")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }

  private void oversizeMessageTest() {
    DisplayMessage smoothCrawl = new DisplayMessage()
        .setDisplayCommand(DisplayCommand.TEXT_CRAWL)
        .setText("some long message")
        .setForegroundRed(64)
        .setForegroundGreen(32)
        .setForegroundBlue(64)
        .setDelay1(40);
    displayConnection.send(smoothCrawl);
  }
}
