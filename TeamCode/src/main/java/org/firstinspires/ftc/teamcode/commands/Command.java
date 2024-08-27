package org.firstinspires.ftc.teamcode.commands;

public interface Command {

    void initialize();

    void execute();

    boolean isDone();

    void end();

}
