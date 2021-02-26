// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousSquare extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousSquare(Drivetrain drivetrain) {
    addCommands(
       new DriveDistanceT(0.5,(0.5*.96),20, drivetrain),
       new TurnGyro(-0.5, 75, drivetrain),
       // new TurnDegrees(-0.5, 75, drivetrain),
       new DriveDistanceT(0.5,(0.5*.96),20, drivetrain),
       new TurnGyro(-0.5, 75, drivetrain),
       // new TurnDegrees(-0.5, 75, drivetrain),
       new DriveDistanceT(0.5,(0.5*.96),20, drivetrain),
       new TurnGyro(-0.5, 75, drivetrain),
       // new TurnDegrees(-0.5, 75, drivetrain),
       new DriveDistanceT(0.5,(0.5*.96),20, drivetrain),
       // new TurnDegrees(-0.5, 75, drivetrain));
       new TurnGyro(-0.5, 75, drivetrain));
      //addCommands(
       //new TurnDegrees(-0.5, 90, drivetrain),
       //new TurnDegrees(0.5, 90, drivetrain));
  }
}
