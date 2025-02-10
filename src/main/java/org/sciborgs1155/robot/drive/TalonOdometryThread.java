package org.sciborgs1155.robot.drive;

public class TalonOdometryThread extends Thread{

    private static TalonOdometryThread instance = null;

    public static TalonOdometryThread getInstance() {
        if (instance == null) {
            instance = new TalonOdometryThread();
        }
        return instance;
    }

    @Override
    public synchronized void start() {
        super.start();
    }

    @Override
    public void run() {
        // TODO write
    }
    
}
