package at.jku.icg.aos_dji_sdkv5.core;

import android.view.Surface;

/**
 * YUV frame decoder - passes frames to native layer for processing.
 * Simplified from decompiled original (JADX failed to decompile onReceive).
 */
public class YuvDecoder {
    public static final String TAG = "YuvDecoder";
    private final AOSManager aosManager;
    private Surface surface;
    private int frames = 0;

    public YuvDecoder(AOSManager aosManager) {
        this.aosManager = aosManager;
    }

    public void onReceive(final int format, final byte[] data, final int width, final int height) {
        int f = this.frames;
        this.frames = f + 1;
        if (f % 3 != 0 || data == null || !this.aosManager.isRunning()) {
            return;
        }
        // Original code passed frames to NativeLib.live_surface_view_receive_imageinfo_withgimbal
        // with telemetry data from AOSManager.pollLiveInfo(). The decompiled version was
        // corrupted (raw bytecode). For LIS_Swarm, video frames go through DroneSwarmStreamData
        // -> native feedFrame() -> RTSP instead, so this path is unused.
    }

    public Surface getSurface() {
        return this.surface;
    }

    public void setSurface(Surface surface) {
        this.surface = surface;
    }
}
