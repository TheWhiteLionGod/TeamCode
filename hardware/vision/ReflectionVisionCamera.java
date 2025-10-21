package org.firstinspires.ftc.teamcode.hardware.vision;

import java.lang.reflect.Method;

public class ReflectionVisionCamera implements VisionCamera {
    private final Object portal;
    private final Method close;

    public ReflectionVisionCamera(Object portal) throws ReflectiveOperationException {
        this.portal = portal;
        this.close = portal.getClass().getMethod("close");
    }

    @Override
    public void close() {
        try { close.invoke(portal); }
        catch (Exception ignored) {}
    }
}
