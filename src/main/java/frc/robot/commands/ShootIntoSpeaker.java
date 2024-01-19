package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
public class ShootIntoSpeaker extends Command {
  /** Creates a new ShootIntoSpeaker. */
  private Shooter shooter;
  public ShootIntoSpeaker(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterSpeed(1.00);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}