// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCurve extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  //private final double m_speedr;
  //private final double m_speedl;
  private final double m_speed;
  private final double m_speed_ratio;

   /**
   * Creates a new DriveDistance. This command will drive your your robot a disired curve at
   * a desired speed.
   *
   * @param speedr The speed at which the robot will drive
   * @param speedl The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   * @param speed The speed at which the robot will drive
   * @param speed_ratio The speed at which the robot will drive
   * 
   */
 
  /** Creates a new DriveCurve. */
  public DriveCurve(double speed, double speed_ratio, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_speed_ratio = speed_ratio;
    m_drive = drive;
    addRequirements(drive);
  }
// Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.tankDrive(0, 0);
    m_drive.resetGyro();
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_speed_ratio > 0)
    {
    m_drive.tankDrive(m_speed, (m_speed * m_speed_ratio));
    }
    else
    {
    m_drive.tankDrive((m_speed * Math.abs(m_speed_ratio)), m_speed); 
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
