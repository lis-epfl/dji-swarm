package com.liscontroller;

import android.app.Application;
import android.content.Context;

import com.secneo.sdk.Helper;

import java.io.IOException;
import java.io.InputStream;

/**
 * DJI MSDK v5 requires Helper.install() in attachBaseContext, before any SDK class is
 * loaded (the same hook as lis-swarm-app's LISApplication). CrashLog goes in FIRST, so
 * a crash inside Helper.install() itself is captured too.
 */
public class LisControllerApplication extends Application {

    @Override
    protected void attachBaseContext(Context base) {
        super.attachBaseContext(base);
        CrashLog.install(base);
        CrashLog.step("MSDK payload assets/sdkclasses.bangcle: " + payload(base));
        CrashLog.step("Helper.install ...");
        Helper.install(this);
        CrashLog.step("Helper.install done");
        CrashLog.reassert("after Helper.install");
    }

    @Override
    public void onCreate() {
        super.onCreate();
        // MSDK's own content provider (BackGroundWatcherInstaller) has run by now.
        CrashLog.reassert("after the SDK's content providers");
        CrashLog.step("Application.onCreate");
    }

    /**
     * The protected MSDK classes Helper.install() loads. Version 1.0 shipped without
     * them and crashed on open; the build now refuses such an APK (app/build.gradle).
     * This line makes a regression obvious on the device as well.
     */
    private static String payload(Context c) {
        try (InputStream in = c.getAssets().open("sdkclasses.bangcle")) {
            return in.available() + " bytes";
        } catch (IOException e) {
            return "MISSING - Helper.install() will fail (see packagingOptions in app/build.gradle)";
        }
    }
}
