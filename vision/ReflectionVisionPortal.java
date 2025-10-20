package org.firstinspires.ftc.teamcode.vision;

import java.lang.reflect.Method;

public class ReflectionVisionPortal implements VisionPortal {
    private final Object portal;
    private final Method close;

    public ReflectionVisionPortal(Object portal) throws ReflectiveOperationException {
        this.portal = portal;
        this.close = portal.getClass().getMethod("close");
    }

    @Override
    public void close() {
        try { close.invoke(portal); }
        catch (Exception ignored) {}
    }
}
