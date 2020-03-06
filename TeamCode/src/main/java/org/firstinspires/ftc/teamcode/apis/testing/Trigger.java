package org.firstinspires.ftc.teamcode.apis.testing;

public class Trigger {
    private long time = 0;

    public Trigger(long waitMs) {
        time = System.currentTimeMillis() + waitMs;
    }

    public boolean getState() {
        return System.currentTimeMillis() > time;
    }
}
