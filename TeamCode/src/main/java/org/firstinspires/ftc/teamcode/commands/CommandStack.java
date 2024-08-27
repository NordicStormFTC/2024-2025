package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.List;

public class CommandStack {

    public static List<Command> commandList = new ArrayList<>();

    public CommandStack() {

    }

    public void add(Command command) {
        commandList.add(command);
    }

    public void run() {
        if (commandList.size() > 0) {
            while (commandList.size() > 0) {
                for (int i = 0; i <= commandList.size(); i++) {
                    commandList.get(i).initialize();
                    while (!commandList.get(i).isDone()) {
                        commandList.get(i).execute();
                    }
                    commandList.get(i).end();
                }
                break;
            }
        }
    }
}
