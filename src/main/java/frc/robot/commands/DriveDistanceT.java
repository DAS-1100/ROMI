// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistanceT extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speedr;
  private final double m_speedl;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speedr The speed at which the robot will drive
   * @param speedl The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistanceT(double speedr,double speedl, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speedr = speedr;
    m_speedl = speedl;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_drive.tankDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_drive.tankDrive(m_speedl, m_speedr);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }
  
  // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
    }
}
