// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpeakerShoot extends Command {
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;
  private Timer timer;
  public AutoSpeakerShoot(ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.intake = intake;
    this.shooter = shooter;
    timer = new Timer();

    addRequirements(intake);
    addRequirements(shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setShooterLSpeed(1.0);
    shooter.setShooterRSpeed(1.0);

    if(timer.get()> .9){
        intake.outtake(.6);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopL();
    shooter.stopR();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.1;
  }
}