package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
public class ReverseShooter extends Command {
  /** Creates a new ShootIntoSpeaker. */
  private Shooter shooter;
  public ReverseShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterRSpeed(.75);
    shooter.setShooterLSpeed(.75);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopR();
    shooter.stopL();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}