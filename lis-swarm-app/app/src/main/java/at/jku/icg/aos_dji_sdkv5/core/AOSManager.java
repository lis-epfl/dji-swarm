package at.jku.icg.aos_dji_sdkv5.core;

import at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfo;
import at.jku.icg.aos_dji_sdkv5.liveinfo.LiveInfoTask;
import java.util.Timer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;

public class AOSManager {
    public AOSActivity aosActivity;
    private Timer liveInfoTimer;
    private boolean running;
    private LiveInfoTask task;
    private final ExecutorService executorService = Executors.newSingleThreadExecutor();
    private final LinkedBlockingQueue<LiveInfo> sensor_info_queue = new LinkedBlockingQueue<>();
    private YuvDecoder yuvDecoder = new YuvDecoder(this);

    public void start() {
        if (this.liveInfoTimer == null) {
            this.liveInfoTimer = new Timer();
            LiveInfoTask liveInfoTask = new LiveInfoTask(this);
            this.task = liveInfoTask;
            liveInfoTask.addListener(liveInfo -> sensor_info_queue.add(liveInfo));
            this.liveInfoTimer.scheduleAtFixedRate(this.task, 200L, 100L);
        }
    }

    public void startProcessingImages() {
        this.running = true;
        this.executorService.execute(() -> {
            while (running) {
                NativeLib.process_images();
            }
        });
    }

    public void stop() {
        this.running = false;
        if (this.liveInfoTimer != null) {
            LiveInfoTask liveInfoTask = this.task;
            if (liveInfoTask != null) {
                liveInfoTask.cancel();
            }
            this.liveInfoTimer.purge();
            this.liveInfoTimer.cancel();
            this.liveInfoTimer = null;
            this.task = null;
        }
        this.sensor_info_queue.clear();
    }

    public void destroy() {
        this.executorService.shutdown();
    }

    public YuvDecoder getYuvDecoder() {
        return this.yuvDecoder;
    }

    public LiveInfoTask getLiveInfoTask() {
        return this.task;
    }

    public LiveInfo pollLiveInfo() {
        try {
            return this.sensor_info_queue.take();
        } catch (InterruptedException unused) {
            return new LiveInfo();
        }
    }

    public boolean isRunning() {
        return this.running;
    }

    public void setRunning(boolean z) {
        this.running = z;
    }
}
