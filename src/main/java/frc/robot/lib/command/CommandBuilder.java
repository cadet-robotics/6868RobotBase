// FRC 340
package frc.robot.lib.command;

import java.util.ConcurrentModificationException;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandBuilder extends Command {
    private Runnable onInit = () -> {};
    private Runnable onExecute = () -> {};
    private Runnable onEnd = () -> {}; 
    private BooleanSupplier isFinished = () -> false;

    public CommandBuilder( Subsystem... requirements ) {
        addRequirements(requirements);
    }

    public CommandBuilder( String name, Subsystem... requirements ) {
        this(requirements);
        setName(name);
    }

    public CommandBuilder onInit( Runnable onInit ) {
        if (this.isScheduled()) throw new ConcurrentModificationException("Cannot change methods of a command while it is scheduled.");
        this.onInit = onInit;
        return this;
    }

    public CommandBuilder onExecute( Runnable onExecute ) {
        if (this.isScheduled()) throw new ConcurrentModificationException("Cannot change methods of a command while it is scheduled.");
        this.onExecute = onExecute;
        return this;
    }

    public CommandBuilder onEnd( Runnable onEnd ) {
        if (this.isScheduled()) throw new ConcurrentModificationException("Cannot change methods of a command while it is scheduled.");
        this.onEnd = onEnd;
        return this;
    }

    public CommandBuilder isFinished(boolean isFinished) {
        if (this.isScheduled()) throw new ConcurrentModificationException("Cannot change methods of a command while it is scheduled.");
        this.isFinished = () -> isFinished;
        return this;
    }

    public CommandBuilder isFinished( BooleanSupplier isFinished ) {
        if (this.isScheduled()) throw new ConcurrentModificationException("Cannot change methods of a command while it is scheduled.");
        this.isFinished = isFinished;
        return this;
    }

    @Override
    public void initialize() {
        onInit.run();
    }  

    @Override
    public void execute() { 
        onExecute.run();
    }

    @Override
    public void end( boolean interrupted ) {
        onEnd.run();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}