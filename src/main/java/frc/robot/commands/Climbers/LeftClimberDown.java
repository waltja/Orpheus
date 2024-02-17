package frc.robot.commands.Climbers;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftClimber;

public class LeftClimberDown extends Command {
  private LeftClimber climber;
  public LeftClimberDown(LeftClimber climber) {
    this.climber = climber;
    addRequirements(climber);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    climber.climberMove(-.5);
  }
  @Override
  public void end(boolean interrupted) {
    climber.climbersStop();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}