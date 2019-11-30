package org.firstinspires.ftc.teamcode.common.utilities;

import android.app.Application;
import android.content.Context;

import java.lang.reflect.Method;

public class Util extends Application {
    public static Context getContext() {
        try {
            final Class<?> activityThreadClass =
                    Class.forName("android.app.ActivityThread");
            //find and load the main activity method
            final Method method = activityThreadClass.getMethod("currentApplication");
            return (Application) method.invoke(null, (Object[]) null);
        } catch (final java.lang.Throwable e) {
            // handle exception
            throw new IllegalArgumentException("No context could be retrieved!");
        }
    }
}
