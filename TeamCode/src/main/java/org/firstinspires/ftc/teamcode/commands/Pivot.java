package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.teamcode.systems.Heading;

public class Pivot implements Command {

    private double degrees;
    private Heading heading;

    public Pivot(double degrees, Heading heading){
        this.degrees = degrees;

    }
    @Override
    public void initialize() {
        degrees = Math.toDegrees(angleWrap(1));
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void end() {

    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }
}
