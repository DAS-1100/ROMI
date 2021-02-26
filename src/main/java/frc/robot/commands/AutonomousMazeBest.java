// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousMazeBest extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous Drive based on distance. This will drive out for a
     * specified distance, turn around and drive back.
     *
     * @param drivetrain The drivetrain subsystem on which this command will run
     */
    public AutonomousMazeBest(Drivetrain drivetrain) {
    addCommands(
      new DriveDistanceT(0.4,(0.4*.98),.5, drivetrain),
      new DriveDistanceT(0.6,(0.6*.98),4.5, drivetrain),
      new DriveCurve(0.6,-.83,16.5, drivetrain),
      new DriveDistanceT(0.6,(0.6*.98),3, drivetrain),
      new DriveCurve(0.6,-.53,8.5, drivetrain),
      new DriveDistanceT(0.5,(0.5*.98),17, drivetrain),
      new DriveCurve(0.6,.52,9, drivetrain),
      new DriveCurve(0.6,.85,19, drivetrain),
      new DriveDistanceT(0.5,(0.5*.98),5, drivetrain),
      new TurnGyro(0.5, 1, drivetrain));
      //addCommands(
    
  }
}
