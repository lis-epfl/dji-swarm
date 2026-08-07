package com.lisswarm;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.Button;
import android.widget.TextView;

import at.jku.icg.aos_dji_sdkv5.core.AOSManager;
import at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData;
import at.jku.icg.aos_dji_sdkv5.core.MQTTEmbedded;
import at.jku.icg.aos_dji_sdkv5.dji.DJIManager;

import dji.sdk.keyvalue.key.AirLinkKey;
import dji.sdk.keyvalue.key.BatteryKey;
import dji.sdk.keyvalue.key.DJIKeyInfo;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.airlink.Bandwidth;
import dji.sdk.keyvalue.value.airlink.ChannelSelectionMode;
import dji.sdk.keyvalue.value.airlink.FrequencyBand;
import dji.sdk.keyvalue.value.airlink.FrequencyInterferenceInfo;
import dji.sdk.keyvalue.value.camera.VideoFrameRate;
import dji.sdk.keyvalue.value.camera.VideoResolution;
import dji.sdk.keyvalue.value.camera.VideoResolutionFrameRate;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.EmptyMsg;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.common.Velocity3D;
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason;
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem;
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam;
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode;
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation;
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.common.video.channel.VideoChannelType;
import dji.v5.common.video.decoder.DecoderOutputMode;
import dji.v5.common.video.decoder.VideoDecoder;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager;
import dji.v5.manager.aircraft.virtualstick.VirtualStickState;
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Main activity for LIS_Swarm. Streams video/telemetry to PC via RTSP (native
 * libs) and receives joystick commands via embedded MQTT broker.
 *
 * Command protocol (received via MQTT):
 *   "VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw"
 *     pitch/roll = world N/E velocity m/s, yaw = yaw RATE deg/s,
 *     throttle = absolute altitude m, gimbal angles = absolute deg.
 *   "ENABLE_VS" / "DISABLE_VS"
 *   "TAKEOFF" / "LAND"
 *   "AIRLINK:band=..:bw=..:video=.." — one-shot radio/camera-stream setup,
 *     NAMED fields, any subset (see applyAirlinkSettings; sent by the PC after
 *     its command channel connects so per-drone radio config lives in the PC's
 *     flocking config)
 *   "LINKDIAG" — read-only: publish one LINKSCAN: line back to the PC
 *
 * Published back to the PC on MQTTEmbedded.DIAG_TOPIC (the only app->PC path
 * besides the RTSP telemetry string, whose native 17-field signature is fixed):
 *   "LINK:sq:down:up"       — cached link quality, 1 Hz, unsolicited
 *   "LINKSCAN:band=..:..."  — full radio config + interference sweep, on demand
 *   "AIRLINKRES:<text>"     — outcome of each AIRLINK: field (applied /
 *                             NOT APPLIED / REJECTED + reason). tvStatus is one
 *                             TextView so the next message overwrites it; the
 *                             PC that asked for the change gets told directly.
 *
 * Operator lockout: the on-screen Disable VS button latches out ALL PC motion
 * commands (VS:, ENABLE_VS, TAKEOFF, LAND) until the on-screen Enable VS
 * button is pressed. AIRLINK:, LINKDIAG and DISABLE_VS pass through while
 * latched (none of them move the aircraft).
 * PC-sent DISABLE_VS does NOT latch (GUI Stop→Start keeps working).
 */
public class SwarmActivity extends Activity {

    private static final String TAG = "LIS_Swarm";
    private static final long VS_SEND_INTERVAL_MS = 50;   // 20 Hz
    private static final long TELEM_UI_INTERVAL_MS = 500;  // 2 Hz UI refresh

    // Existing video/telemetry pipeline
    private DJIManager djiManager;
    private AOSManager aosManager;
    public DroneSwarmStreamData droneSwarmStreamData;
    private MQTTEmbedded mqttEmbedded;

    // Virtual stick state
    private volatile double vsPitch = 0;
    private volatile double vsRoll = 0;
    private volatile double vsYaw = 0;          // yaw RATE, deg/s (ANGULAR_VELOCITY mode)
    private volatile double vsThrottle = 0;
    private volatile double cmdGimbalPitch = -90;
    private volatile double cmdGimbalYaw = 0;
    private volatile boolean vsActive = false;

    // Operator lockout latch: set by the local Disable VS button, cleared ONLY
    // by the local Enable VS button. While set, PC motion commands (VS:,
    // ENABLE_VS, TAKEOFF, LAND) are ignored — the PC streams VS: at 20 Hz for
    // the life of its controller and its QoS-1 one-shots can arrive late after
    // a link blip, so a plain disableVirtualStick() call is not an operator
    // override (the next incoming message re-fills the command cache within
    // 50 ms, and a queued ENABLE_VS can silently re-arm). PC-initiated
    // DISABLE_VS never sets this latch, so normal GUI Stop→Start is unaffected.
    private volatile boolean pcLockout = false;
    private long lockoutDropCount = 0;              // MQTT thread ++, UI thread resets; log-only
    private volatile long lastLockoutLogMs = 0;
    private volatile long lastLockoutDisableRetryMs = 0;
    private static final long LOCKOUT_LOG_INTERVAL_MS = 2000;
    private static final long LOCKOUT_DISABLE_RETRY_MS = 1000;

    // volatile: written by DJI's state listener, read by the VS send timer
    // (the lockout retry checks it cross-thread)
    private volatile VirtualStickState currentVsState;
    private Timer vsSendTimer;
    private Timer telemUiTimer;

    // Telemetry values (updated by KeyManager callbacks)
    private volatile double telemLat = 0, telemLon = 0, telemAlt = 0;
    private volatile double telemHeading = 0;
    private volatile double telemPitch = 0, telemRoll = 0, telemYaw = 0;
    private volatile double telemGimbalPitch = 0, telemGimbalRoll = 0, telemGimbalYaw = 0;
    private volatile double telemVx = 0, telemVy = 0, telemVz = 0;
    private volatile int telemSatCount = 0;
    private volatile int telemBatteryPercent = -1;   // -1 = not yet reported
    private volatile int telemSignalQuality = -1;     // 0-100, -1 = not yet reported
    private volatile int telemDownLinkQuality = -1;   // 0-100, -1 = not yet reported
    private volatile int telemUpLinkQuality = -1;     // 0-100, -1 = not yet reported

    // Link diagnostics published back to the PC on MQTTEmbedded.DIAG_TOPIC.
    private static final String LINK_STATUS_PREFIX = "LINK:";
    private static final String LINK_SCAN_PREFIX = "LINKSCAN:";
    private static final String AIRLINK_RESULT_PREFIX = "AIRLINKRES:";
    private static final long LINK_STATUS_INTERVAL_MS = 1000;   // 1 Hz
    private Timer linkStatusTimer;

    // Video
    private SurfaceView surfaceVideo;

    // UI
    private TextView tvStatus;
    private TextView tvTelemetry;
    private TextView tvIp;
    private TextView tvVsState;
    private TextView tvTelemGps;
    private TextView tvTelemAttitude;
    private TextView tvTelemGimbal;
    private TextView tvTelemVelocity;
    private TextView tvBattery;
    private TextView tvLink;
    private Button btnStartRtsp;
    private Button btnEnableVs;
    private Button btnDisableVs;
    private Handler uiHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_swarm);

        uiHandler = new Handler(Looper.getMainLooper());
        initUI();
        initVideoAndTelemetry();
        initMQTT();
        initVirtualStick();
        startVsSendLoop();
        startTelemetryListeners();
        startTelemUiLoop();
        startLinkStatusLoop();
    }

    private void initUI() {
        tvStatus = findViewById(R.id.tv_status);
        tvTelemetry = findViewById(R.id.tv_telemetry);
        tvIp = findViewById(R.id.tv_ip);
        tvVsState = findViewById(R.id.tv_vs_state);
        tvTelemGps = findViewById(R.id.tv_telem_gps);
        tvTelemAttitude = findViewById(R.id.tv_telem_attitude);
        tvTelemGimbal = findViewById(R.id.tv_telem_gimbal);
        tvTelemVelocity = findViewById(R.id.tv_telem_velocity);
        tvBattery = findViewById(R.id.tv_battery);
        tvLink = findViewById(R.id.tv_link);
        btnStartRtsp = findViewById(R.id.btn_start_rtsp);
        btnEnableVs = findViewById(R.id.btn_enable_vs);
        btnDisableVs = findViewById(R.id.btn_disable_vs);
        surfaceVideo = findViewById(R.id.surface_video);

        btnStartRtsp.setOnClickListener(v -> onStartRtspClicked());
        btnEnableVs.setOnClickListener(v -> onLocalEnableClicked());
        btnDisableVs.setOnClickListener(v -> onLocalDisableClicked());

        // Create DJI VideoDecoder once the surface is available
        surfaceVideo.getHolder().addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                Log.i(TAG, "Surface created, starting VideoDecoder");
                if (djiManager != null && djiManager.videodecoder != null) {
                    djiManager.videodecoder.onPause();
                    djiManager.videodecoder.destroy();
                }
                if (djiManager != null) {
                    djiManager.videodecoder = new VideoDecoder(
                        getApplicationContext(),
                        VideoChannelType.PRIMARY_STREAM_CHANNEL,
                        DecoderOutputMode.SURFACE_MODE,
                        holder,
                        surfaceVideo.getWidth(),
                        surfaceVideo.getHeight(),
                        true);
                    Log.i(TAG, "VideoDecoder created");
                }
            }

            @Override
            public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
            }

            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                Log.i(TAG, "Surface destroyed");
                if (djiManager != null && djiManager.videodecoder != null) {
                    djiManager.videodecoder.onPause();
                    djiManager.videodecoder.destroy();
                    djiManager.videodecoder = null;
                }
            }
        });

        String ethIp = getEth0Ip();
        tvIp.setText("eth0: " + ethIp);
        tvStatus.setText("Initializing... RTSP not started yet");
    }

    // ========== RTSP Control ==========

    /**
     * User presses "Start RTSP" once video is confirmed working on screen.
     * This starts the native RTSP server with the correct detected video format,
     * and enables frame feeding.
     */
    private void onStartRtspClicked() {
        if (droneSwarmStreamData == null) {
            updateStatus("Video pipeline not ready");
            return;
        }
        boolean ok = droneSwarmStreamData.startRtspServer();
        String ethIp = getEth0Ip();
        if (ok) {
            btnStartRtsp.setEnabled(false);
            btnStartRtsp.setText("RTSP ON");
            updateStatus("RTSP server started on " + ethIp + ":8554");
            tvIp.setText("RTSP: rtsp://video:video@" + ethIp + ":8554");
        } else {
            updateStatus("RTSP start FAILED — check logcat");
        }
    }

    // ========== Video & Telemetry (existing pipeline) ==========

    private void initVideoAndTelemetry() {
        aosManager = new AOSManager();
        aosManager.aosActivity = null;

        // Mark AOSManager as running so getTelemetryData() actually collects data.
        aosManager.setRunning(true);

        djiManager = new DJIManager(this);

        droneSwarmStreamData = new DroneSwarmStreamData(djiManager);
        droneSwarmStreamData.aosManager = aosManager;
        // Start with RTSP feed DISABLED — user enables it via the button
        droneSwarmStreamData.EnableRtspStream = false;
        djiManager.droneSwarmStreamData = droneSwarmStreamData;

        try {
            droneSwarmStreamData.start();
            updateStatus("Video pipeline started. Press 'Start RTSP' to stream.");
        } catch (Exception e) {
            Log.e(TAG, "Failed to start video pipeline", e);
            updateStatus("Video start failed: " + e.getMessage());
        }

        aosManager.start();
    }

    // ========== MQTT ==========

    private void initMQTT() {
        mqttEmbedded = new MQTTEmbedded(aosManager, getFilesDir().getAbsolutePath());
        mqttEmbedded.setCommandListener(this::onCommandReceived);
        try {
            mqttEmbedded.run();
            updateStatus("MQTT broker started on :1883");
        } catch (Exception e) {
            Log.e(TAG, "Failed to start MQTT broker", e);
            updateStatus("MQTT FAILED: " + e.getMessage());
        }
    }

    public void onCommandReceived(String command) {
        // DIAGNOSTIC: log every MQTT command landing on the app
        Log.v(TAG, "MQTT recv: " + command + "  ts=" + System.currentTimeMillis());
        if (command == null || command.isEmpty()) return;

        // Moquette's intercept handler fires for ANY publish including our own
        // broker-internal ones, so the link diagnostics we publish come straight
        // back here. Drop them before anything else looks at them.
        if (command.startsWith(LINK_STATUS_PREFIX)
                || command.startsWith(LINK_SCAN_PREFIX)
                || command.startsWith(AIRLINK_RESULT_PREFIX)) {
            return;
        }

        // Operator lockout: drop every motion command from the PC (VS: stream,
        // ENABLE_VS, TAKEOFF, LAND). DISABLE_VS stays honored (redundant but
        // harmless, and it never clears the latch); AIRLINK: stays honored
        // (radio/camera config, no motion).
        if (pcLockout && (command.startsWith("VS:") || command.equals("ENABLE_VS")
                || command.equals("TAKEOFF") || command.equals("LAND"))) {
            lockoutDropCount++;
            long now = System.currentTimeMillis();
            if (now - lastLockoutLogMs >= LOCKOUT_LOG_INTERVAL_MS) {
                lastLockoutLogMs = now;
                Log.i(TAG, "PC lockout: dropped " + lockoutDropCount
                    + " PC motion commands (latest: " + command + ")");
            }
            return;
        }

        if (command.startsWith("VS:")) {
            String[] parts = command.substring(3).split(":");
            if (parts.length >= 4) {
                vsPitch = Double.parseDouble(parts[0]);
                vsRoll = Double.parseDouble(parts[1]);
                vsYaw = Double.parseDouble(parts[2]);
                vsThrottle = Double.parseDouble(parts[3]);
                if (parts.length >= 5) cmdGimbalPitch = Double.parseDouble(parts[4]);
                if (parts.length >= 6) cmdGimbalYaw = Double.parseDouble(parts[5]);
            }
        } else if (command.equals("ENABLE_VS")) {
            enableVirtualStick();
        } else if (command.equals("DISABLE_VS")) {
            disableVirtualStick();
        } else if (command.equals("TAKEOFF")) {
            performTakeoff();
        } else if (command.equals("LAND")) {
            performLanding();
        } else if (command.startsWith("AIRLINK:")) {
            applyAirlinkSettings(command.substring("AIRLINK:".length()));
        } else if (command.equals("LINKDIAG")) {
            publishLinkScan();
        }
    }

    // ========== AirLink management ==========

    /**
     * "AIRLINK:band=&lt;b&gt;:bw=&lt;MHz&gt;:video=&lt;WxH@fps&gt;" — with ten aircraft/RC
     * links sharing the spectrum, DJI's per-link auto selection has no view of
     * the whole fleet, so the PC assigns radio settings deterministically.
     *
     * Fields are NAMED, not positional, and any subset may be sent; '-' or
     * absent = leave unchanged. Named fields matter here because this command
     * evolves: an RC still running an older APK parses an unknown name as an
     * unknown field and says so, instead of silently applying a new value to
     * whatever used to sit in that position.
     *   band   2G4 | 5G8 | DUAL — splitting the fleet across the two bands
     *          halves the number of contenders per band
     *   bw     40 | 20 | 10 | 5 — AirLink channel bandwidth in MHz. THE most
     *          useful knob for a crowded site: it narrows the spectrum each
     *          link actually occupies (video=... only lowers the encoded
     *          bitrate inside whatever channel width is in use), and unlike
     *          channel pinning it works in AUTO channel-selection mode.
     *   video  camera stream cap, e.g. 1920x1080@24
     *
     * There is deliberately NO manual-channel field: DJI does not support
     * manual image-transmission channel selection on the Mini 3 Pro, so the
     * old channel= path could only ever report a rejection. Use bw + band, and
     * LINKDIAG to see the interference picture the aircraft's own auto
     * selection is working against.
     *
     * Every set is read back and surfaced via updateStatus/logcat, so firmware
     * that locks a key is visible on the RC screen during a bench test.
     */
    private void applyAirlinkSettings(String spec) {
        try {
            String modeTok = "", bandTok = "", bwTok = "", videoTok = "";
            for (String field : spec.split(":")) {
                field = field.trim();
                int eq = field.indexOf('=');
                if (eq < 0) continue;              // ignore unknown/legacy tokens
                String key = field.substring(0, eq).trim();
                String val = field.substring(eq + 1).trim();
                if (key.equals("mode"))       modeTok = val;
                else if (key.equals("band"))  bandTok = val;
                else if (key.equals("bw"))    bwTok = val;
                else if (key.equals("video")) videoTok = val;
                else airlinkStatus("AIRLINK: ignoring unknown field '" + key + "'");
            }

            // mode= must complete BEFORE the rest: the 2026-08-07 bench run
            // showed the aircraft accepting bandwidth in AUTO, holding it for
            // under 3 s, then reverting when its channel selection next ran.
            // DJI documents bandwidth (and band) as manual-mode-only, so the
            // radio fields are chained behind the mode change rather than
            // fired alongside it. Without mode= the behaviour is unchanged.
            if (!modeTok.isEmpty() && !modeTok.equals("-")) {
                final ChannelSelectionMode mode =
                    modeTok.equals("MANUAL") ? ChannelSelectionMode.MANUAL :
                    modeTok.equals("AUTO") ? ChannelSelectionMode.AUTO : null;
                if (mode == null) {
                    airlinkStatus("AIRLINK: unknown mode '" + modeTok + "'");
                } else {
                    final String fBand = bandTok, fBw = bwTok, fVideo = videoTok;
                    KeyManager.getInstance().setValue(
                        KeyTools.createKey(AirLinkKey.KeyChannelSelectionMode),
                        mode,
                        new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onSuccess() {
                                airlinkStatus("AIRLINK mode=" + mode + " accepted");
                                applyRadioFields(fBand, fBw, fVideo);
                            }

                            @Override
                            public void onFailure(IDJIError error) {
                                // Still apply the rest: the outcome reports
                                // then show whether the radio fields need the
                                // mode at all, which is the open question.
                                airlinkStatus("AIRLINK mode=" + mode
                                    + " REJECTED: " + error.description());
                                applyRadioFields(fBand, fBw, fVideo);
                            }
                        });
                    return;
                }
            }
            applyRadioFields(bandTok, bwTok, videoTok);
        } catch (Exception e) {
            // Never let a malformed one-shot kill the command listener
            airlinkStatus("AIRLINK parse failed for '" + spec + "': " + e);
        }
    }

    /** band / bw / video, applied after any mode= change has settled. */
    private void applyRadioFields(String bandTok, String bwTok, String videoTok) {
        try {
            if (!bandTok.isEmpty() && !bandTok.equals("-")) {
                FrequencyBand band =
                    bandTok.equals("2G4") ? FrequencyBand.BAND_2_DOT_4G :
                    bandTok.equals("5G8") ? FrequencyBand.BAND_5_DOT_8G :
                    bandTok.equals("DUAL") ? FrequencyBand.BAND_DUAL : null;
                if (band == null) {
                    airlinkStatus("AIRLINK: unknown band '" + bandTok + "'");
                } else {
                    // No implicit mode change here: band alone is what DJI Fly
                    // offers on this airframe with channel selection on auto.
                    // Callers who need manual mode ask for it with mode=.
                    setAirlinkKeyAndVerify("band", AirLinkKey.KeyFrequencyBand, band);
                }
            }

            if (!bwTok.isEmpty() && !bwTok.equals("-")) {
                Bandwidth bw =
                    bwTok.equals("40") ? Bandwidth.BANDWIDTH_40MHZ :
                    bwTok.equals("20") ? Bandwidth.BANDWIDTH_20MHZ :
                    bwTok.equals("10") ? Bandwidth.BANDWIDTH_10MHZ :
                    bwTok.equals("5")  ? Bandwidth.BANDWIDTH_5MHZ : null;
                if (bw == null) {
                    airlinkStatus("AIRLINK: unknown bandwidth '" + bwTok + "' MHz");
                } else {
                    setAirlinkKeyAndVerify("bandwidth", AirLinkKey.KeyBandwidth, bw);
                }
            }

            if (!videoTok.isEmpty() && !videoTok.equals("-")) {
                int at = videoTok.indexOf('@');
                VideoResolution res = VideoResolution.valueOf(
                    "RESOLUTION_" + videoTok.substring(0, at));
                VideoFrameRate rate = VideoFrameRate.valueOf(
                    "RATE_" + videoTok.substring(at + 1) + "FPS");
                droneSwarmStreamData.setVideoResolution(
                    new VideoResolutionFrameRate(res, rate));
                airlinkStatus("AIRLINK video -> " + videoTok);
            }
        } catch (Exception e) {
            // Never let a malformed one-shot kill the command listener. Also
            // catches the video= enum lookups, whose valueOf() throws on a
            // resolution/fps the SDK does not name.
            airlinkStatus("AIRLINK field apply failed (band='" + bandTok
                + "' bw='" + bwTok + "' video='" + videoTok + "'): " + e);
        }
    }

    /**
     * Report an AirLink outcome to the RC screen, logcat AND the PC.
     *
     * The PC leg is the one that matters operationally: tvStatus is a single
     * TextView, so the next message overwrites this one — during the
     * 2026-08-07 bench run the bandwidth verdict was gone from the screen
     * within 3 s, replaced by the LINKDIAG line. At ten aircraft, reading ten
     * RC screens (or ten adb logcats) to find out whether a set took is not a
     * workable answer; the PC asked for the change, so the PC gets told.
     */
    private void airlinkStatus(String msg) {
        updateStatus(msg);                       // screen + logcat as before
        MQTTEmbedded.publishDiagnostic(AIRLINK_RESULT_PREFIX + msg);
    }

    private <T> void setAirlinkKeyAndVerify(String what, DJIKeyInfo<T> keyInfo, T value) {
        KeyManager.getInstance().setValue(
            KeyTools.createKey(keyInfo), value,
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    // "Accepted" only means the SDK took the call. Read it back
                    // and report BOTH values: a set that succeeds and leaves
                    // the old value in place is a real firmware behaviour and
                    // is invisible if only the request is reported.
                    readBackAirlinkKey(what, keyInfo, String.valueOf(value));
                }

                @Override
                public void onFailure(IDJIError error) {
                    airlinkStatus("AIRLINK " + what + "=" + value
                        + " REJECTED: " + error.description());
                }
            });
    }

    private <T> void readBackAirlinkKey(String what, DJIKeyInfo<T> keyInfo,
                                        final String asked) {
        KeyManager.getInstance().getValue(
            KeyTools.createKey(keyInfo),
            new CommonCallbacks.CompletionCallbackWithParam<T>() {
                @Override
                public void onSuccess(T readBack) {
                    boolean took = String.valueOf(readBack).equals(asked);
                    airlinkStatus("AIRLINK " + what + " set accepted, reads "
                        + readBack
                        + (took ? " (applied)"
                                : " — asked " + asked + ", NOT APPLIED"));
                }

                @Override
                public void onFailure(IDJIError error) {
                    airlinkStatus("AIRLINK " + what
                        + " set OK but read-back failed: " + error.description());
                }
            });
    }

    // ========== Telemetry Display ==========

    /**
     * Register KeyManager listeners to continuously receive drone telemetry.
     * These update volatile fields that the UI timer reads.
     */
    private void startTelemetryListeners() {
        // GPS location
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), this,
            (LocationCoordinate3D oldVal, LocationCoordinate3D newVal) -> {
                if (newVal != null) {
                    telemLat = newVal.getLatitude();
                    telemLon = newVal.getLongitude();
                    telemAlt = newVal.getAltitude();
                }
            });

        // Compass heading
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyCompassHeading), this,
            (Double oldVal, Double newVal) -> {
                if (newVal != null) telemHeading = newVal;
            });

        // Aircraft attitude
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), this,
            (Attitude oldVal, Attitude newVal) -> {
                if (newVal != null) {
                    telemPitch = newVal.getPitch();
                    telemRoll = newVal.getRoll();
                    telemYaw = newVal.getYaw();
                }
            });

        // Gimbal attitude
        KeyManager.getInstance().listen(
            KeyTools.createKey(GimbalKey.KeyGimbalAttitude), this,
            (Attitude oldVal, Attitude newVal) -> {
                if (newVal != null) {
                    telemGimbalPitch = newVal.getPitch();
                    telemGimbalRoll = newVal.getRoll();
                    telemGimbalYaw = newVal.getYaw();
                }
            });

        // Velocity
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity), this,
            (Velocity3D oldVal, Velocity3D newVal) -> {
                if (newVal != null) {
                    telemVx = newVal.getX();
                    telemVy = newVal.getY();
                    telemVz = newVal.getZ();
                }
            });

        // Satellite count
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemSatCount = newVal;
            });

        // Battery charge remaining (percent)
        KeyManager.getInstance().listen(
            KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemBatteryPercent = newVal;
            });

        // Air link signal quality (0-100), i.e. connection quality to the RC.
        // DJI's own reading of this scale: <40 poor, 40-60 normal, >60 good.
        KeyManager.getInstance().listen(
            KeyTools.createKey(AirLinkKey.KeySignalQuality), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemSignalQuality = newVal;
            });

        // Directional link quality (0-100). Worth having separately from the
        // combined signal quality: the video downlink and the command uplink
        // degrade independently, and with ten co-located links it is the
        // downlink that saturates first.
        KeyManager.getInstance().listen(
            KeyTools.createKey(AirLinkKey.KeyDownLinkQuality), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemDownLinkQuality = newVal;
            });
        KeyManager.getInstance().listen(
            KeyTools.createKey(AirLinkKey.KeyUpLinkQuality), this,
            (Integer oldVal, Integer newVal) -> {
                if (newVal != null) telemUpLinkQuality = newVal;
            });
    }

    // ========== Link diagnostics (read-only, app -> PC) ==========

    /**
     * Publish the cached link-quality numbers to the PC at 1 Hz.
     *
     * Reads ONLY the cached values the listeners above already maintain — no
     * KeyManager.getValue() round-trips — because per-frame key polling is
     * exactly what was removed from DroneSwarmStreamData for adding RF chatter
     * without adding freshness. Payload:
     *   LINK:&lt;signalQuality&gt;:&lt;downLinkQuality&gt;:&lt;upLinkQuality&gt;
     * with -1 for "not reported yet".
     */
    private void startLinkStatusLoop() {
        linkStatusTimer = new Timer("LinkStatus");
        linkStatusTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                MQTTEmbedded.publishDiagnostic(String.format(Locale.US,
                    LINK_STATUS_PREFIX + "%d:%d:%d",
                    telemSignalQuality, telemDownLinkQuality, telemUpLinkQuality));
            }
        }, LINK_STATUS_INTERVAL_MS, LINK_STATUS_INTERVAL_MS);
    }

    /**
     * One-shot answer to the PC's "LINKDIAG" command: read the current radio
     * configuration plus the aircraft's own interference measurement and
     * publish it as one line.
     *
     * This is the band scan that DJI's auto channel selection is silently
     * working against — KeyFrequencyInterference returns an RSSI per frequency
     * bucket, which is the only way to tell "the site is noisy" apart from
     * "our config is wrong" without external spectrum kit.
     *
     * getValue() is used here (not listeners) precisely because it is a
     * one-shot: the operator asks, the aircraft answers once. Each read is
     * independent and best-effort, so a key the firmware locks just reports
     * "?" for that field rather than failing the whole scan. Field order:
     *   LINKSCAN:band=..:mode=..:bw=..:freq=..:sq=..:down=..:up=..:if=..
     * where if= is a comma list of &lt;fromMHz&gt;-&lt;toMHz&gt;@&lt;rssi&gt;.
     */
    private void publishLinkScan() {
        // One shared buffer filled by several async callbacks, then published
        // once when the last one lands. StringBuilder is not thread-safe and
        // the callbacks arrive on SDK threads, so the parts array + a counter
        // guarded by the activity monitor does the assembly instead.
        final String[] parts = new String[4];    // band, mode, bw, freq
        final int[] remaining = {4};

        readScanField(parts, remaining, 0, "band", AirLinkKey.KeyFrequencyBand);
        readScanField(parts, remaining, 1, "mode", AirLinkKey.KeyChannelSelectionMode);
        readScanField(parts, remaining, 2, "bw", AirLinkKey.KeyBandwidth);
        readScanField(parts, remaining, 3, "freq", AirLinkKey.KeyFrequencyPoint);
    }

    /** Read one scan field; when the last outstanding read lands, publish. */
    private <T> void readScanField(final String[] parts, final int[] remaining,
                                   final int slot, final String name,
                                   DJIKeyInfo<T> keyInfo) {
        KeyManager.getInstance().getValue(
            KeyTools.createKey(keyInfo),
            new CommonCallbacks.CompletionCallbackWithParam<T>() {
                @Override
                public void onSuccess(T value) {
                    finish(name + "=" + value);
                }

                @Override
                public void onFailure(IDJIError error) {
                    // Locked/unsupported key: report it as unknown rather than
                    // losing the whole scan.
                    Log.i(TAG, "LINKDIAG " + name + " read failed: "
                        + error.description());
                    finish(name + "=?");
                }

                private void finish(String text) {
                    boolean last;
                    synchronized (SwarmActivity.this) {
                        parts[slot] = text;
                        last = (--remaining[0] == 0);
                    }
                    if (last) readInterferenceAndPublish(parts);
                }
            });
    }

    /** Final step of a scan: append the interference sweep and publish. */
    private void readInterferenceAndPublish(final String[] parts) {
        final String head = LINK_SCAN_PREFIX
            + parts[0] + ":" + parts[1] + ":" + parts[2] + ":" + parts[3]
            + String.format(Locale.US, ":sq=%d:down=%d:up=%d",
                telemSignalQuality, telemDownLinkQuality, telemUpLinkQuality);

        KeyManager.getInstance().getValue(
            KeyTools.createKey(AirLinkKey.KeyFrequencyInterference),
            new CommonCallbacks.CompletionCallbackWithParam<List<FrequencyInterferenceInfo>>() {
                @Override
                public void onSuccess(List<FrequencyInterferenceInfo> infos) {
                    StringBuilder sb = new StringBuilder(head).append(":if=");
                    if (infos != null) {
                        boolean first = true;
                        for (FrequencyInterferenceInfo info : infos) {
                            if (info == null) continue;
                            if (!first) sb.append(',');
                            first = false;
                            sb.append(String.format(Locale.US, "%.0f-%.0f@%d",
                                nz(info.getFrequencyFrom()), nz(info.getFrequencyTo()),
                                info.getRssi() == null ? 0 : info.getRssi()));
                        }
                    }
                    emit(sb.toString());
                }

                @Override
                public void onFailure(IDJIError error) {
                    Log.i(TAG, "LINKDIAG interference read failed: "
                        + error.description());
                    emit(head + ":if=?");
                }

                private void emit(String payload) {
                    MQTTEmbedded.publishDiagnostic(payload);
                    updateStatus("LINKDIAG -> " + payload);
                }
            });
    }

    private static double nz(Double v) {
        return v == null ? 0.0 : v;
    }

    /**
     * Periodic UI update showing live drone telemetry on screen.
     */
    private void startTelemUiLoop() {
        telemUiTimer = new Timer("TelemUI");
        telemUiTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                uiHandler.post(() -> {
                    tvBattery.setText(telemBatteryPercent < 0
                        ? "BAT: --%"
                        : String.format(Locale.US, "BAT: %d%%", telemBatteryPercent));
                    tvBattery.setTextColor(batteryColor(telemBatteryPercent));

                    tvLink.setText(telemSignalQuality < 0
                        ? "LINK: --"
                        : String.format(Locale.US, "LINK: %d%%", telemSignalQuality));
                    tvLink.setTextColor(signalColor(telemSignalQuality));

                    tvTelemGps.setText(String.format(Locale.US,
                        "GPS: %.6f, %.6f  Alt:%.1fm\nHdg:%.1f  Sat:%d",
                        telemLat, telemLon, telemAlt, telemHeading, telemSatCount));

                    tvTelemAttitude.setText(String.format(Locale.US,
                        "ATT: P:%.1f R:%.1f Y:%.1f",
                        telemPitch, telemRoll, telemYaw));

                    tvTelemGimbal.setText(String.format(Locale.US,
                        "GMB: P:%.1f R:%.1f Y:%.1f",
                        telemGimbalPitch, telemGimbalRoll, telemGimbalYaw));

                    tvTelemVelocity.setText(String.format(Locale.US,
                        "VEL: X:%.2f Y:%.2f Z:%.2f\nRTSP: %s  Frames: %d",
                        telemVx, telemVy, telemVz,
                        droneSwarmStreamData != null && droneSwarmStreamData.RtspIsRunning ? "ON" : "OFF",
                        droneSwarmStreamData != null ? droneSwarmStreamData.getRtspFramesFed() : 0));
                });
            }
        }, 500, TELEM_UI_INTERVAL_MS);
    }

    // ========== Virtual Stick ==========

    private void initVirtualStick() {
        VirtualStickManager.getInstance().setVirtualStickStateListener(
            new VirtualStickStateListener() {
                @Override
                public void onVirtualStickStateUpdate(VirtualStickState state) {
                    currentVsState = state;
                    vsActive = state.isVirtualStickEnable();
                    droneSwarmStreamData.virtualstickonoff = vsActive ? 1.0 : 0.0;
                    refreshVsStateLabel();
                }

                @Override
                public void onChangeReasonUpdate(FlightControlAuthorityChangeReason reason) {
                    Log.i(TAG, "Flight authority change: " + reason.name());
                }
            });
    }

    // ========== Local operator buttons (RC screen) ==========

    /**
     * Disable VS button: engage the PC lockout latch, close the send-loop gate
     * immediately (waiting for DJI's async state listener would let PC commands
     * keep reaching the FC for a beat after the press), then disable VS. The
     * latch stays on until the local Enable VS button is pressed — a PC GUI
     * Start will NOT re-arm this aircraft while latched.
     */
    private void onLocalDisableClicked() {
        pcLockout = true;
        lockoutDropCount = 0;
        vsActive = false;   // eager; onVirtualStickStateUpdate stays the source of truth
        disableVirtualStick();
        updateStatus("PC LOCKOUT ON — ignoring PC motion commands until Enable VS");
        refreshVsStateLabel();
    }

    /** Enable VS button: clear the lockout latch and re-enable VS. */
    private void onLocalEnableClicked() {
        pcLockout = false;
        refreshVsStateLabel();
        enableVirtualStick();
    }

    private void refreshVsStateLabel() {
        final String text = String.format("VS: %s%s",
            vsActive ? "ENABLED" : "DISABLED",
            pcLockout ? " | PC LOCKED OUT" : "");
        uiHandler.post(() -> tvVsState.setText(text));
    }

    private void enableVirtualStick() {
        VirtualStickManager.getInstance().enableVirtualStick(
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    Log.i(TAG, "Virtual stick enabled");
                    VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(true);
                    updateStatus("Virtual stick ENABLED");
                }

                @Override
                public void onFailure(IDJIError error) {
                    Log.e(TAG, "Enable VS failed: " + error.description());
                    updateStatus("VS enable failed: " + error.description());
                }
            });
    }

    private void disableVirtualStick() {
        vsPitch = 0;
        vsRoll = 0;
        vsYaw = 0;

        VirtualStickManager.getInstance().disableVirtualStick(
            new CommonCallbacks.CompletionCallback() {
                @Override
                public void onSuccess() {
                    Log.i(TAG, "Virtual stick disabled");
                    updateStatus("Virtual stick DISABLED");
                }

                @Override
                public void onFailure(IDJIError error) {
                    Log.e(TAG, "Disable VS failed: " + error.description());
                    updateStatus("VS disable FAILED: " + error.description());
                }
            });
    }

    private void startVsSendLoop() {
        vsSendTimer = new Timer("VS_Send");
        vsSendTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (pcLockout) {
                    // Belt-and-braces while latched: if VS somehow reports
                    // enabled again (initial disable failed silently, or a
                    // stray external re-enable), keep re-issuing the disable
                    // at ~1 Hz until the FC confirms it is off.
                    VirtualStickState s = currentVsState;
                    if (s != null && s.isVirtualStickEnable()) {
                        long now = System.currentTimeMillis();
                        if (now - lastLockoutDisableRetryMs >= LOCKOUT_DISABLE_RETRY_MS) {
                            lastLockoutDisableRetryMs = now;
                            Log.w(TAG, "PC lockout: VS still enabled, retrying disable");
                            disableVirtualStick();
                        }
                    }
                    return;
                }
                if (!vsActive) return;

                VirtualStickFlightControlParam param = new VirtualStickFlightControlParam();
                param.setRollPitchCoordinateSystem(FlightCoordinateSystem.GROUND);
                param.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
                // Yaw is RATE-controlled (deg/s), not absolute angle. ANGLE mode
                // makes the FC's position controller chase a stepped heading
                // setpoint, which is visibly jerky both while turning and while
                // holding. The PC side runs a heading-hold P controller and sends
                // a smooth yaw rate; vsYaw is deg/s. (See the yaw note in CLAUDE.md.)
                param.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
                param.setVerticalControlMode(VerticalControlMode.POSITION);
                // AXIS NOTE: in GROUND + VELOCITY on this Mini 3 Pro / MSDK v5,
                // the DJI pitch axis moves the aircraft EAST and the roll axis
                // moves it NORTH — the transpose of the naive assumption. Verified
                // by the 2026-07-05 rotation-check: with pitch<-north/roll<-east
                // every drone flew east on a north command and north on an east
                // command, identically at west/north/south headings (heading-
                // independent => world frame, but N/E swapped). So feed the
                // protocol's EAST field to setPitch and its NORTH field to setRoll.
                param.setPitch(vsRoll);   // vsRoll  = protocol EAST  field -> DJI pitch(E)
                param.setRoll(vsPitch);   // vsPitch = protocol NORTH field -> DJI roll(N)
                param.setYaw(vsYaw);
                param.setVerticalThrottle(vsThrottle);

                VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param);
                sendGimbalCommand(cmdGimbalPitch, cmdGimbalYaw);

                uiHandler.post(() -> tvTelemetry.setText(String.format(Locale.US,
                    "Cmd: P=%.1f R=%.1f Y=%.1f T=%.1f  Gimbal: P=%.1f Y=%.1f",
                    vsPitch, vsRoll, vsYaw, vsThrottle, cmdGimbalPitch, cmdGimbalYaw)));
            }
        }, 0, VS_SEND_INTERVAL_MS);
    }

    private void sendGimbalCommand(double pitch, double yaw) {
        GimbalAngleRotation rotation = new GimbalAngleRotation();
        rotation.setMode(GimbalAngleRotationMode.ABSOLUTE_ANGLE);
        rotation.setPitch(pitch);
        rotation.setYaw(yaw);

        KeyManager.getInstance().performAction(
            KeyTools.createKey(GimbalKey.KeyRotateByAngle),
            rotation,
            new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                @Override public void onSuccess(EmptyMsg msg) {}
                @Override public void onFailure(IDJIError error) {}
            });
    }

    private void performTakeoff() {
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyStartTakeoff),
            null,
            new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                @Override
                public void onSuccess(EmptyMsg msg) { updateStatus("Takeoff initiated"); }
                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("Takeoff failed: " + error.description());
                }
            });
    }

    private void performLanding() {
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding),
            null,
            new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                @Override
                public void onSuccess(EmptyMsg msg) { updateStatus("Landing initiated"); }
                @Override
                public void onFailure(IDJIError error) {
                    updateStatus("Landing failed: " + error.description());
                }
            });
    }

    // ========== Helpers ==========

    private void updateStatus(String msg) {
        Log.i(TAG, msg);
        uiHandler.post(() -> tvStatus.setText(msg));
    }

    /** Green >50%, amber 20-50%, red <20%, grey if unknown. */
    private static int batteryColor(int percent) {
        if (percent < 0) return 0xFFCCCCCC;
        if (percent < 20) return 0xFFFF4444;
        if (percent < 50) return 0xFFFFCC00;
        return 0xFF00FF00;
    }

    /** DJI's own reading of KeySignalQuality: <40 poor, 40-60 normal, >60
     *  good. Grey if not reported yet. */
    private static int signalColor(int quality) {
        if (quality < 0) return 0xFFCCCCCC;
        if (quality < 40) return 0xFFFF4444;
        if (quality <= 60) return 0xFFFFCC00;
        return 0xFF00FF00;
    }

    /**
     * Returns the IPv4 address of eth0 (ethernet). Falls back to any
     * non-loopback IPv4 if eth0 has no address.
     */
    private String getEth0Ip() {
        String fallbackIp = null;
        try {
            Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
            if (interfaces == null) return "no interfaces";
            for (NetworkInterface ni : Collections.list(interfaces)) {
                for (InetAddress addr : Collections.list(ni.getInetAddresses())) {
                    if (addr.isLoopbackAddress() || !(addr instanceof Inet4Address)) continue;
                    String ip = addr.getHostAddress();
                    // Prefer eth0 (wired ethernet on RC Pro)
                    if (ni.getName().startsWith("eth")) {
                        return ip;
                    }
                    if (fallbackIp == null) {
                        fallbackIp = ip;
                    }
                }
            }
        } catch (SocketException e) {
            Log.e(TAG, "Failed to get IP", e);
        }
        return fallbackIp != null ? fallbackIp : "no IP";
    }

    // ========== Lifecycle ==========

    @Override
    protected void onDestroy() {
        if (vsSendTimer != null) vsSendTimer.cancel();
        if (telemUiTimer != null) telemUiTimer.cancel();
        if (linkStatusTimer != null) linkStatusTimer.cancel();

        if (vsActive) {
            vsPitch = 0; vsRoll = 0;
            VirtualStickManager.getInstance().disableVirtualStick(
                new CommonCallbacks.CompletionCallback() {
                    @Override public void onSuccess() {}
                    @Override public void onFailure(IDJIError e) {}
                });
        }

        if (droneSwarmStreamData != null) droneSwarmStreamData.stop();
        if (aosManager != null) {
            aosManager.stop();
            aosManager.destroy();
        }
        if (djiManager != null) djiManager.onDestroy();
        if (mqttEmbedded != null) mqttEmbedded.stop();

        KeyManager.getInstance().cancelListen(this);
        super.onDestroy();
    }
}
