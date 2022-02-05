// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Seek extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain dt;
  private double tv = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Seek(DriveTrain dt) {
    this.dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //connect limelight to tv
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      dt.tankDrive(1, -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      dt.tankDrive(0, 0);
      //Add seeek command
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tv != 0;
  }
}
