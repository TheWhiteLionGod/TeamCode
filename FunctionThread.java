package org.firstinspires.ftc.teamcode;

public class FunctionThread extends Thread {
    ThreadInterface execute;
    ThreadInterface cleanup;

    public FunctionThread(ThreadInterface execute, ThreadInterface cleanup) {
        this.execute = execute;
        this.cleanup = cleanup;
    }

    @Override
    public void run() {
        try {
            // Runs the Function
            execute.run();
        }
        catch (InterruptedException ignored) {
            // Stopping Thread if Interrupted
        }
        finally {
            try {
                cleanup.run();
            } catch (InterruptedException ignored) {

            }
        }
    }
}
