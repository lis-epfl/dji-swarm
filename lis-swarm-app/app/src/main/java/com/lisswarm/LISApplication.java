package com.lisswarm;

import android.app.Application;
import android.content.Context;
import com.secneo.sdk.Helper;

/**
 * Application entry point. DJI SDK v5 requires Helper.install() in attachBaseContext
 * before any SDK classes can be loaded.
 */
public class LISApplication extends Application {

    @Override
    protected void attachBaseContext(Context context) {
        super.attachBaseContext(context);
        Helper.install(this);
    }

    @Override
    public void onCreate() {
        super.onCreate();
    }
}
