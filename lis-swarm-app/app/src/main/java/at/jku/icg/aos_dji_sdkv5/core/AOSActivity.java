package at.jku.icg.aos_dji_sdkv5.core;

import android.app.Activity;
import android.os.Environment;
import java.io.File;

/**
 * Stub for the original AOSActivity. SwarmActivity replaces all functionality.
 * This class exists only because AOSManager declares an AOSActivity field
 * and LiveInfoTask references AOSActivity.baseDir.
 */
public class AOSActivity extends Activity {
    public static File baseDir = new File(Environment.getExternalStorageDirectory(), "DJI_AOS");
}
