package com.liscontroller;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ServiceInfo;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.os.PowerManager;
import android.provider.Settings;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.concurrent.atomic.AtomicBoolean;

import dji.v5.common.error.IDJIError;
import dji.v5.common.register.DJISDKInitEvent;
import dji.v5.manager.SDKManager;
import dji.v5.manager.interfaces.SDKManagerCallback;

/**
 * Owns everything that must outlive the activity: MSDK init + registration, the
 * read-only RcInputReader, the UDP stream and a partial wake lock. It is a foreground
 * service, so the stream keeps running with the screen off or another app on top.
 * Removing the app from recents stops it, with a bye, so the PC goes stale at once
 * rather than after its timeout.
 *
 * It starts in STAGES so a crash can be bisected (safe mode, MainActivity):
 *   1  foreground service + UDP stream (no MSDK at all)
 *   2  + MSDK init and registration
 *   3  + the key listeners (normal operation)
 * A later start request can only raise the stage. START_NOT_STICKY: after a crash the
 * system does NOT restart it, which is what turned one bug into "keeps stopping".
 */
public class StreamService extends Service implements UdpStreamServer.InfoSource {
    static final String EXTRA_STAGE = "stage";
    static final int STAGE_SERVICE = 1;
    static final int STAGE_SDK = 2;
    static final int STAGE_FULL = 3;

    private static final String CHANNEL = "stream";
    private static final int NOTE_ID = 1;
    private static final long REGISTER_RETRY_MS = 15000;

    // MSDK state is PROCESS-wide: swiping the app away destroys the service but keeps the
    // process, and SDKManager.init must run once per process. Per-instance flags here left a
    // recreated service waiting forever after a failed registration (1.2, 2026-09-25).
    private static final AtomicBoolean sdkInitStarted = new AtomicBoolean(false);
    private static volatile boolean sdkInitComplete;       // INITIALIZE_COMPLETE seen
    private static volatile String registrationFailure;    // last failure, null once registered

    /** The running instance, for the UI (same process). */
    static volatile StreamService instance;

    private final Handler main = new Handler(Looper.getMainLooper());
    private RcInputReader reader;
    private UdpStreamServer server;
    private PowerManager.WakeLock wakeLock;
    private volatile RfStatus rf = RfStatus.UNKNOWN;
    private volatile String ethIp;
    private String appVersion = "?";

    volatile int stage;
    volatile String sdkStatus = "SDK: not started";
    volatile boolean sdkRegistered;
    volatile boolean sdkFailed;
    volatile String serverError;

    @Override
    public void onCreate() {
        super.onCreate();
        CrashLog.step("service onCreate");
        instance = this;
        startInForeground();
        try {
            appVersion = getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
        } catch (PackageManager.NameNotFoundException ignored) {
        }
        PowerManager pm = (PowerManager) getSystemService(POWER_SERVICE);
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "liscontroller:stream");
        wakeLock.setReferenceCounted(false);
        wakeLock.acquire();

        reader = new RcInputReader(() -> {
            UdpStreamServer s = server;
            if (s != null) s.wake();
        });
        server = new UdpStreamServer(reader, this);
        try {
            server.start();
            CrashLog.step("UDP stream listening on :" + Protocol.PORT);
        } catch (SocketException e) {
            serverError = "cannot open UDP :" + Protocol.PORT + " - " + e.getMessage();
            CrashLog.error("UDP server start", e);
        }
        main.post(pollNetwork);
        CrashLog.step("service onCreate done");
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        // Every startForegroundService() must be answered by startForeground() within
        // seconds, including a repeat start from a recreated activity. Re-posting the
        // same notification is idempotent.
        startInForeground();
        int want = intent == null ? STAGE_FULL : intent.getIntExtra(EXTRA_STAGE, STAGE_FULL);
        raiseStage(want);
        return START_NOT_STICKY;
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onTaskRemoved(Intent rootIntent) {
        CrashLog.step("app removed from recents - stopping the stream");
        stopSelf();
    }

    @Override
    public void onDestroy() {
        CrashLog.step("service onDestroy");
        main.removeCallbacks(pollNetwork);
        main.removeCallbacks(registerRetry);
        if (server != null) server.stop("app stopped");
        if (reader != null) reader.stop();
        if (wakeLock != null && wakeLock.isHeld()) wakeLock.release();
        instance = null;
        super.onDestroy();
    }

    // --- for the UI ------------------------------------------------------------------

    RcInputReader.Snapshot snapshot() {
        return reader.snapshot(true);
    }

    UdpStreamServer server() {
        return server;
    }

    /** After a failed registration (the first one needs internet). The button. */
    void retryRegistration() {
        main.removeCallbacks(registerRetry);
        registerNow("retry button");
    }

    private void registerNow(String why) {
        sdkStatus = "SDK: registering (" + why + ")";
        CrashLog.step("SDK registerApp (" + why + ")");
        AsyncTask.execute(() -> {
            try {
                SDKManager.getInstance().registerApp();
            } catch (RuntimeException | LinkageError e) {
                CrashLog.error("SDK registerApp (" + why + ")", e);
                sdkFailed = true;
                sdkStatus = "SDK: registerApp threw " + e;
            }
        });
    }

    /** A failed registration retries by itself, so it goes through once there is internet. */
    private final Runnable registerRetry = new Runnable() {
        @Override
        public void run() {
            if (instance == StreamService.this && !sdkRegistered) registerNow("auto-retry");
        }
    };

    // --- UdpStreamServer.InfoSource -------------------------------------------------

    @Override
    public RfStatus rf() {
        return rf;
    }

    @Override
    public String ethIp() {
        return ethIp;
    }

    @Override
    public String appVersion() {
        return appVersion;
    }

    // --- internals -------------------------------------------------------------------

    /** Main thread. Only ever raises the stage. */
    private void raiseStage(int want) {
        if (want <= stage) return;
        stage = want;
        CrashLog.step("service stage -> " + stage);
        if (stage < STAGE_SDK) {
            sdkStatus = "SDK: not started (safe-mode stage " + stage + ")";
            return;
        }
        initSdk();
        if (stage >= STAGE_FULL && sdkRegistered) startReader();
    }

    private void initSdk() {
        boolean registered;
        try {
            registered = SDKManager.getInstance().isRegistered();
        } catch (RuntimeException | LinkageError e) {
            registered = false;     // MSDK not initialized in this process yet
        }
        if (registered) {
            onRegistered();     // the service was recreated in a live process
            return;
        }
        if (!sdkInitStarted.compareAndSet(false, true)) {
            // Init already ran in this process: this is a recreated service. If it completed
            // without registering, register again now rather than wait for nothing.
            if (sdkInitComplete) {
                sdkFailed = registrationFailure != null;
                registerNow("service recreated");
            } else {
                sdkStatus = "SDK: init in progress";
            }
            return;
        }
        sdkStatus = "SDK: initializing";
        final Context app = getApplicationContext();
        AsyncTask.execute(() -> {
            try {
                CrashLog.step("SDKManager.init ...");
                SDKManager.getInstance().init(app, new SdkCallback());
                CrashLog.step("SDKManager.init returned");
                CrashLog.reassert("after SDKManager.init");
            } catch (RuntimeException | LinkageError e) {
                CrashLog.error("SDKManager.init", e);
                StreamService s = instance;
                if (s != null) {
                    s.sdkFailed = true;
                    s.sdkStatus = "SDK: init threw " + e;
                }
            }
        });
    }

    /** Static, and it always goes through `instance`: it may outlive a service object. */
    private static final class SdkCallback implements SDKManagerCallback {
        @Override
        public void onRegisterSuccess() {
            CrashLog.step("SDK registered");
            registrationFailure = null;
            StreamService s = instance;
            if (s != null) s.main.post(s::onRegistered);
        }

        @Override
        public void onRegisterFailure(IDJIError error) {
            String why = error == null ? "?" : error.description();
            CrashLog.step("SDK registration FAILED: " + why + " - retrying every "
                    + REGISTER_RETRY_MS / 1000 + " s");
            registrationFailure = why;
            StreamService s = instance;
            if (s != null) {
                s.sdkFailed = true;
                s.sdkStatus = "SDK: registration FAILED - " + why
                        + " (the first registration needs internet; retrying every "
                        + REGISTER_RETRY_MS / 1000 + " s)";
                s.main.removeCallbacks(s.registerRetry);
                s.main.postDelayed(s.registerRetry, REGISTER_RETRY_MS);
            }
        }

        @Override
        public void onProductConnect(int productId) {
            // Never expected on this RF-quiet joystick; the reader's interlock blocks
            // the stream if it happens.
            CrashLog.step("PRODUCT CONNECTED: " + productId);
        }

        @Override
        public void onProductDisconnect(int productId) {
            CrashLog.step("product disconnected: " + productId);
        }

        @Override
        public void onProductChanged(int productId) {
            CrashLog.step("product changed: " + productId);
        }

        @Override
        public void onInitProcess(DJISDKInitEvent event, int totalProcess) {
            CrashLog.step("SDK init event " + event + " " + totalProcess + "%");
            StreamService s = instance;
            if (s != null && !s.sdkRegistered) s.sdkStatus = "SDK: " + event + " " + totalProcess + "%";
            if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                sdkInitComplete = true;
                try {
                    CrashLog.step("SDK registerApp ...");
                    SDKManager.getInstance().registerApp();
                } catch (RuntimeException | LinkageError e) {
                    CrashLog.error("SDK registerApp", e);
                    if (s != null) {
                        s.sdkFailed = true;
                        s.sdkStatus = "SDK: registerApp threw " + e;
                    }
                }
            }
        }

        @Override
        public void onDatabaseDownloadProgress(long current, long total) {
        }
    }

    /** Main thread. */
    private void onRegistered() {
        if (instance != this) return;
        CrashLog.reassert("after SDK registration");
        sdkRegistered = true;
        sdkFailed = false;
        if (stage >= STAGE_FULL) {
            sdkStatus = "SDK: registered";
            startReader();
        } else {
            sdkStatus = "SDK: registered - key reading not started (safe-mode stage " + stage + ")";
        }
    }

    private void startReader() {
        try {
            reader.start();
        } catch (RuntimeException | LinkageError e) {
            CrashLog.error("reader.start", e);
        }
    }

    private final Runnable pollNetwork = new Runnable() {
        @Override
        public void run() {
            rf = RfStatus.read(StreamService.this);
            ethIp = eth0Ip();
            pollGamepadGate();
            main.postDelayed(this, 1000);
        }
    };

    // DJI's gate on the RC's gamepad (see GamepadInput), read the way DJI's own framework
    // reads it (com.dji.DjiServicesHelper.enableMotion, com.dji.comkey.ComKeyHelper).
    static final String GATE_GAME_MODE = "dji_lab_game_mode";
    static final String GATE_LEFT = "dji_motion_via_left_joystick_enabled";
    static final String GATE_RIGHT = "dji_motion_via_right_joystick_enabled";
    private String lastGate;

    private void pollGamepadGate() {
        Boolean game = null, left = null, right = null;
        try {
            game = Settings.Global.getInt(getContentResolver(), GATE_GAME_MODE, 0) != 0;
            left = "1".equals(Settings.Global.getString(getContentResolver(), GATE_LEFT));
            right = "1".equals(Settings.Global.getString(getContentResolver(), GATE_RIGHT));
        } catch (RuntimeException e) {
            // Unreadable: GamepadInput then relies on its MSDK witness alone.
        }
        GamepadInput.INSTANCE.gate(game, left, right);
        String g = GATE_GAME_MODE + "=" + game + " " + GATE_LEFT + "=" + left + " " + GATE_RIGHT + "=" + right;
        if (!g.equals(lastGate)) {
            lastGate = g;
            boolean open = Boolean.TRUE.equals(game) && Boolean.TRUE.equals(left) && Boolean.TRUE.equals(right);
            CrashLog.step("DJI gamepad gate: " + g + (open ? " - the gamepad reaches this app"
                    : " - DJI drops the gamepad for this app, so the sticks come from MSDK (~10 Hz);"
                    + " run rc-joystick/android/enable-gamepad.ps1"));
        }
    }

    /** The IPv4 of eth0 (the RC's USB Ethernet), else any non-loopback IPv4. */
    private static String eth0Ip() {
        String fallback = null;
        try {
            Enumeration<NetworkInterface> ifs = NetworkInterface.getNetworkInterfaces();
            if (ifs == null) return null;
            for (NetworkInterface ni : Collections.list(ifs)) {
                for (InetAddress a : Collections.list(ni.getInetAddresses())) {
                    if (a.isLoopbackAddress() || !(a instanceof Inet4Address)) continue;
                    if (ni.getName().startsWith("eth")) return a.getHostAddress();
                    if (fallback == null) fallback = a.getHostAddress();
                }
            }
        } catch (SocketException ignored) {
        }
        return fallback;
    }

    private void startInForeground() {
        NotificationManager nm = getSystemService(NotificationManager.class);
        nm.createNotificationChannel(new NotificationChannel(
                CHANNEL, "Joystick stream", NotificationManager.IMPORTANCE_LOW));
        PendingIntent open = PendingIntent.getActivity(this, 0,
                new Intent(this, MainActivity.class), PendingIntent.FLAG_IMMUTABLE);
        Notification n = new Notification.Builder(this, CHANNEL)
                .setContentTitle(getString(R.string.app_name))
                .setContentText("Streaming RC inputs on UDP :" + Protocol.PORT)
                .setSmallIcon(R.drawable.ic_launcher)
                .setContentIntent(open)
                .setOngoing(true)
                .build();
        if (Build.VERSION.SDK_INT >= 29) {
            startForeground(NOTE_ID, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE);
        } else {
            startForeground(NOTE_ID, n);
        }
    }
}
