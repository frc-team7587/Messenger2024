// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.metuchenmomentum.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {
        
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
//Button bindings
// A-intake goes to ground and then starts intaking in
// Y-intake and shooter pivot to handoff position
// B-manually pivot shooter up
// X-manually pivot shooter down
// Left Trigger-score in amp after handoff
// Right Trigger-shoot directly from intake if in handoff position
// Left Bumper-perform handoff
// Right Bumper-speaker shoot with note already in indexer
// Start-intake and shooter go to zero/starting position
// D-Pad Up-Shooter to handoff position
// D-Pad Down-Shooter to amp position
