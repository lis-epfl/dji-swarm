package com.liscontroller;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.input.InputManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.view.Gravity;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Render-only. It grants permissions, starts StreamService, and draws what the
 * service's reader sees at 10 Hz. Everything that must keep running lives in the
 * service.
 *
 * It also feeds GamepadInput: the RC's built-in gamepad is the fast (~70 Hz) source for the
 * sticks and dials, and Android delivers joystick events only to the FOCUSED window, which
 * is why this lives here and not in the service. The events are consumed here, so Android
 * never turns stick motion into D-pad focus navigation of this screen. While the activity
 * is not in front, the stream carries MSDK's (~10 Hz) values instead.
 *
 * SAFE MODE (rc-joystick/android/DEBUGGING.md) is entered automatically when the last
 * run left a crash report, or on request (adb shell am start -n
 * com.liscontroller/.MainActivity --ez safe true). Nothing starts by itself: the report
 * and the previous run's last steps are shown, and the stage buttons start the service
 * one piece at a time (1 stream only, 2 + MSDK, 3 + key reading), which bisects the crash.
 */
public class MainActivity extends Activity implements InputManager.InputDeviceListener {
    static final String EXTRA_SAFE = "safe";
    /** The RC Pro's built-in gamepad, "DJI embedded joystick" (README). */
    static final int GAMEPAD_VENDOR = 0x2ca3, GAMEPAD_PRODUCT = 0x1501;
    private static final int REQ_PERMS = 7;
    private static final int C_OK = 0xFF2E7D32, C_WAIT = 0xFFEF6C00, C_BAD = 0xFFC62828,
            C_IDLE = 0xFF37474F;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private TextView status, rfBanner, linkedBanner, diag;
    private StickPadView padL, padR;
    private DialBarView dialL, dialR;
    private final TextView[] lamps = new TextView[Protocol.BUTTONS.length];
    private Button retry, grant;
    private View safeRow;
    private InputManager inputManager;
    private boolean safe;
    private int pendingStage;              // stage to start once permissions allow; 0 = none
    private String version = "?";
    private String crashReport, prevTrace;
    private String lastUiError;

    private final ArrayDeque<String> events = new ArrayDeque<>();
    private final SimpleDateFormat clock = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US);
    private String devicesText = "";
    private String tracedDevices;
    // Gamepad reports per second, traced while a control moves: the fast side of the
    // gamepad-vs-MSDK comparison (RcInputReader.logRates traces the MSDK side).
    private int gamepadReports, foreignReports, gamepadLinesTraced;
    private long lastGamepadTick;
    private static final String[] MOTION_KEYS = {"lh", "lv", "rh", "rv", "l", "r"};  // MSDK stat names
    private long lastMotionLog, lastDiag, lastRateAt;
    private final Map<String, Long> prevCounts = new HashMap<>();
    private final Map<String, Double> rates = new HashMap<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        safe = getIntent().getBooleanExtra(EXTRA_SAFE, false) || CrashLog.hasCrash(this);
        CrashLog.step("MainActivity.onCreate" + (safe ? " - SAFE MODE, nothing auto-starts" : ""));
        try {
            version = getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
        } catch (PackageManager.NameNotFoundException ignored) {
        }
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        // Not a keyboard (IME) target. On Android 10, joystick MotionEvents are not pointer
        // events, so ViewRootImpl offers them to a bound IME (Gboard on this RC) before the
        // view hierarchy. That was NOT what blocked the gamepad (DJI's framework was, see
        // GamepadInput), but this app has no text input, so the IME is kept out of the path.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_ALT_FOCUSABLE_IM);
        status = findViewById(R.id.status);
        rfBanner = findViewById(R.id.rf_banner);
        linkedBanner = findViewById(R.id.linked_banner);
        diag = findViewById(R.id.diag);
        padL = findViewById(R.id.pad_left);
        padR = findViewById(R.id.pad_right);
        dialL = findViewById(R.id.dial_left);
        dialR = findViewById(R.id.dial_right);
        retry = findViewById(R.id.btn_retry);
        grant = findViewById(R.id.btn_grant);
        buildLamps();

        findViewById(R.id.btn_stop).setOnClickListener(v -> {
            stopService(new Intent(this, StreamService.class));   // sends bye
            finishAndRemoveTask();
        });
        retry.setOnClickListener(v -> {
            StreamService s = StreamService.instance;
            if (s != null) s.retryRegistration();
        });
        grant.setOnClickListener(v -> ensurePermissionsThenStart());
        safeRow = findViewById(R.id.safe_row);
        findViewById(R.id.btn_stage1).setOnClickListener(v -> requestStage(StreamService.STAGE_SERVICE));
        findViewById(R.id.btn_stage2).setOnClickListener(v -> requestStage(StreamService.STAGE_SDK));
        findViewById(R.id.btn_stage3).setOnClickListener(v -> requestStage(StreamService.STAGE_FULL));
        findViewById(R.id.btn_clear).setOnClickListener(v -> {
            CrashLog.clearCrash(this);
            safe = false;
            loadReports();
            requestStage(StreamService.STAGE_FULL);
        });
        inputManager = (InputManager) getSystemService(INPUT_SERVICE);
        loadReports();
        if (safe) {
            ensurePermissionsThenStart();       // asks for permissions, starts nothing
        } else {
            requestStage(StreamService.STAGE_FULL);
        }
    }

    private void loadReports() {
        crashReport = CrashLog.readCrash(this);
        prevTrace = CrashLog.readPreviousTrace(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
        inputManager.registerInputDeviceListener(this, ui);
        refreshDevices();
        ui.post(refresh);
    }

    @Override
    protected void onPause() {
        ui.removeCallbacks(refresh);
        inputManager.unregisterInputDeviceListener(this);
        GamepadInput.INSTANCE.focus(false);     // the gamepad goes elsewhere now: back to MSDK
        super.onPause();
    }

    // --- permissions + service ------------------------------------------------------

    /** The runtime ("dangerous") subset of lis-swarm-app's list; the rest are install-time. */
    private static List<String> runtimePermissions() {
        List<String> p = new ArrayList<>(Arrays.asList(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.READ_PHONE_STATE));
        if (Build.VERSION.SDK_INT < 30) p.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (Build.VERSION.SDK_INT < 33) p.add(Manifest.permission.READ_EXTERNAL_STORAGE);
        return p;
    }

    private List<String> missing() {
        List<String> out = new ArrayList<>();
        for (String p : runtimePermissions()) {
            if (ContextCompat.checkSelfPermission(this, p) != PackageManager.PERMISSION_GRANTED) {
                out.add(p.substring(p.lastIndexOf('.') + 1));
            }
        }
        return out;
    }

    /** Start the service up to `stage` (StreamService.STAGE_*), after any permission prompt. */
    private void requestStage(int stage) {
        pendingStage = stage;
        ensurePermissionsThenStart();
    }

    private void ensurePermissionsThenStart() {
        List<String> need = new ArrayList<>();
        for (String p : runtimePermissions()) {
            if (ContextCompat.checkSelfPermission(this, p) != PackageManager.PERMISSION_GRANTED) need.add(p);
        }
        if (need.isEmpty()) {
            if (pendingStage > 0) startStage(pendingStage);
        } else {
            CrashLog.step("requesting permissions: " + need.size());
            ActivityCompat.requestPermissions(this, need.toArray(new String[0]), REQ_PERMS);
        }
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] perms, int[] results) {
        super.onRequestPermissionsResult(code, perms, results);
        List<String> miss = missing();
        CrashLog.step("permissions result, still missing: " + miss);
        if (code == REQ_PERMS && miss.isEmpty() && pendingStage > 0) startStage(pendingStage);
    }

    private void startStage(int stage) {
        CrashLog.step("starting the stream service, stage " + stage);
        startForegroundService(new Intent(this, StreamService.class)
                .putExtra(StreamService.EXTRA_STAGE, stage));
    }

    // --- rendering ------------------------------------------------------------------

    private void buildLamps() {
        LinearLayout row1 = findViewById(R.id.lamps_row1);
        LinearLayout row2 = findViewById(R.id.lamps_row2);
        float d = getResources().getDisplayMetrics().density;
        for (int i = 0; i < lamps.length; i++) {
            TextView t = new TextView(this);
            t.setGravity(Gravity.CENTER);
            t.setTextSize(11);
            t.setPadding(0, (int) (3 * d), 0, (int) (3 * d));
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0,
                    LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
            lp.setMargins((int) (2 * d), (int) (2 * d), (int) (2 * d), (int) (2 * d));
            (i < 7 ? row1 : row2).addView(t, lp);
            lamps[i] = t;
        }
    }

    private final Runnable refresh = new Runnable() {
        @Override
        public void run() {
            GamepadInput.INSTANCE.uiTick();     // the gamepad path's "this screen is live"
            tickGamepadRate();
            renderSafely();
            ui.postDelayed(this, 100);
        }
    };

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        GamepadInput.INSTANCE.focus(hasFocus);
        CrashLog.step("window focus " + (hasFocus ? "gained" : "lost"));
        if (hasFocus) immersive();
    }

    /**
     * Hide the system bars, sticky: an edge swipe only peeks at them, so a stray swipe does
     * not take focus (and with it the gamepad) away from this screen.
     */
    private void immersive() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                | View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION);
    }

    /**
     * Once a second, while a control moves: gamepad reports/s and the stick source (UI
     * thread only). Also traced while MSDK reports motion, so "the gamepad delivered
     * nothing" is an explicit zero in the trace, not an absence.
     */
    private void tickGamepadRate() {
        long now = SystemClock.elapsedRealtime();
        if (now - lastGamepadTick < 1000) return;
        boolean msdkMoving = false;
        for (String k : MOTION_KEYS) {
            Double r = rates.get(k);
            if (r != null && r > 0) msdkMoving = true;
        }
        if ((gamepadReports > 0 || foreignReports > 0 || msdkMoving) && gamepadLinesTraced < 300) {
            gamepadLinesTraced++;
            StreamService svc = StreamService.instance;
            RcInputReader.Snapshot s = svc == null ? null : svc.snapshot();
            CrashLog.step("gamepad reports/s: " + gamepadReports
                    + (s == null || s.gamepad == null ? "" : ", " + gapText(s.gamepad.gapMs))
                    + " (stick source " + (s == null ? "-" : s.stickSrc) + ")"
                    + (foreignReports > 0 ? ", " + foreignReports + " from another joystick (ignored)" : ""));
        }
        gamepadReports = 0;
        foreignReports = 0;
        lastGamepadTick = now;
    }

    /** A display bug must never take the joystick down: it is shown and logged instead. */
    private void renderSafely() {
        try {
            render();
        } catch (RuntimeException e) {
            String msg = e.toString();
            if (!msg.equals(lastUiError)) {
                lastUiError = msg;
                CrashLog.error("UI render", e);
            }
            bar(C_BAD, "UI error: " + msg);
        }
    }

    private void render() {
        long now = SystemClock.elapsedRealtime();
        String app = getString(R.string.app_name) + " " + version + (safe ? "   SAFE MODE" : "");
        safeRow.setVisibility(safe ? View.VISIBLE : View.GONE);
        StreamService svc = StreamService.instance;
        if (svc == null) {
            List<String> miss = missing();
            bar(safe ? C_WAIT : C_IDLE, app + "   " + (!miss.isEmpty() ? "permissions missing: " + miss
                    : safe ? "nothing started - use the stage buttons"
                    : "starting the stream service..."));
            grant.setVisibility(miss.isEmpty() ? View.GONE : View.VISIBLE);
            if (now - lastDiag >= 500) {
                lastDiag = now;
                diag.setText(reportText() + "(the stream service is not running)\n\n"
                        + "THIS RUN, last steps\n" + CrashLog.recent(15));
            }
            return;
        }
        grant.setVisibility(View.GONE);
        RcInputReader.Snapshot s = svc.snapshot();
        UdpStreamServer srv = svc.server();
        List<UdpStreamServer.Subscriber> subs = srv != null ? srv.subscribers()
                : Collections.<UdpStreamServer.Subscriber>emptyList();

        padL.set("LEFT", s.sticks[0], s.sticks[1]);
        padR.set("RIGHT", s.sticks[2], s.sticks[3]);
        dialL.set("left dial", s.dials[0]);
        dialR.set("right dial", s.dials[1]);
        for (int i = 0; i < lamps.length; i++) lamp(lamps[i], Protocol.BUTTONS[i], s.buttons[i], s.presses[i]);
        linkedBanner.setVisibility(s.aircraftLinked ? View.VISIBLE : View.GONE);
        if (s.aircraftLinked) linkedBanner.setText(getString(R.string.linked_banner, s.linkReason));

        List<String> warn = svc.rf().warnings();
        rfBanner.setVisibility(warn.isEmpty() ? View.GONE : View.VISIBLE);
        if (!warn.isEmpty()) {
            rfBanner.setText("RF: " + join(warn) + "  -  switch it off (Settings; Location > "
                    + "Wi-Fi and Bluetooth scanning)");
        }

        String eth = svc.ethIp();
        String head = app + "   " + (eth == null ? "no Ethernet IP" : eth) + ":" + Protocol.PORT + "   ";
        String reason = s.notOkReason();
        if (svc.serverError != null) {
            bar(C_BAD, head + svc.serverError);
        } else if (!svc.sdkRegistered) {
            bar(svc.sdkFailed ? C_BAD : C_WAIT, head + svc.sdkStatus);
        } else if (reason != null) {
            bar(C_BAD, head + "NOT OK - " + reason);
        } else if (subs.isEmpty()) {
            bar(C_WAIT, head + "READY - no PC subscribed - " + sourceText(s));
        } else if (s.slowReason() != null) {
            // Streaming, but degraded: the PC gets ~10 Hz sticks. Amber, with the reason.
            bar(C_WAIT, head + "STREAMING to " + subs.size() + " @ " + srv.currentRate + " Hz - "
                    + sourceText(s));
        } else {
            bar(C_OK, head + "STREAMING to " + subs.size() + " @ " + srv.currentRate + " Hz - "
                    + sourceText(s));
        }
        retry.setVisibility(svc.sdkFailed ? View.VISIBLE : View.GONE);

        if (now - lastRateAt >= 1000) {
            double dt = lastRateAt == 0 ? 0 : (now - lastRateAt) / 1000.0;
            for (RcInputReader.KeyStat k : s.stats) {
                Long prev = prevCounts.get(k.name);
                if (prev != null && dt > 0) rates.put(k.name, (k.count - prev) / dt);
                prevCounts.put(k.name, k.count);
            }
            lastRateAt = now;
        }
        if (now - lastDiag >= 500) {
            lastDiag = now;
            diag.setText(reportText() + diagText(svc, s, srv, subs, now));
        }
    }

    /** The crash report and, in safe mode, where the previous run stopped. Empty when clean. */
    private String reportText() {
        StringBuilder b = new StringBuilder();
        if (crashReport != null) {
            b.append("=== LAST CRASH (crash.txt) - press 'Clear report' once read ===\n")
                    .append(crashReport).append('\n');
        }
        if (safe && prevTrace != null) {
            b.append("=== PREVIOUS RUN, last steps (trace.prev.txt) ===\n")
                    .append(tail(prevTrace, 40)).append('\n');
        }
        return b.toString();
    }

    private static String tail(String text, int lines) {
        String[] all = text.split("\n");
        StringBuilder b = new StringBuilder();
        for (int i = Math.max(0, all.length - lines); i < all.length; i++) {
            b.append("  ").append(all[i]).append('\n');
        }
        return b.toString();
    }

    private void bar(int color, String text) {
        status.setBackgroundColor(color);
        status.setText(text);
    }

    private static void lamp(TextView t, String name, Boolean down, int presses) {
        t.setText(name + "\n" + (down == null ? "n/a" : String.valueOf(presses)));
        if (down == null) {                 // not served by this RC
            t.setBackgroundColor(0xFF1C2429);
            t.setTextColor(0xFF546E7A);
        } else if (down) {
            t.setBackgroundColor(0xFF2E7D32);
            t.setTextColor(0xFFFFFFFF);
        } else {
            t.setBackgroundColor(0xFF37474F);
            t.setTextColor(0xFFECEFF1);
        }
    }

    private String diagText(StreamService svc, RcInputReader.Snapshot s, UdpStreamServer srv,
                            List<UdpStreamServer.Subscriber> subs, long now) {
        StringBuilder b = new StringBuilder();
        b.append(svc.sdkStatus).append("   (service stage ").append(svc.stage).append(")\n");
        b.append(String.format(Locale.US, "RC      %s  sn %s  fw %s  battery %s%%  stick %s  pairing %s%n",
                s.rcType, s.sn, s.fw, s.batteryPct, s.stickMode, s.pairing));
        b.append(String.format(Locale.US, "LINK    rc_conn %s  product %s (type %s)  fc %s  still_linked %s%n",
                s.rcConnected, s.productConnected, s.productType, s.fcConnected, s.stillLinked));
        b.append(String.format(Locale.US, "AIRCRAFT %s%n", s.aircraftLinked
                ? "LINKED: " + s.linkReason
                : "not linked (product connected alone is the RC itself on an RC Pro)"));
        b.append(String.format(Locale.US, "rc_ok   %s%s%n", s.rcOk,
                s.rcOk ? "" : "  (" + s.notOkReason() + ")"));
        GamepadInput.Reading g = s.gamepad;
        if (g != null) {
            b.append(String.format(Locale.US, "STICKS  from %s | gamepad %d reports/s, %s, last %s | focus %s | DJI gate: %s%n",
                    sourceText(s), g.hz, gapText(g.gapMs), g.ageMs < 0 ? "never" : g.ageMs + " ms ago",
                    g.focused ? "yes" : "NO", gateText(g)));
            b.append("        gamepad ").append(wireText(g.sticks, g.dials))
                    .append("   msdk ").append(wireText(s.msdkSticks, s.msdkDials)).append('\n');
            b.append("        cross-check vs MSDK (both still): ").append(checkText(g)).append('\n');
        }
        b.append(String.format(Locale.US, "PROBE   liveness read of %s: ok %d fail %d, last %d ms, age %d ms%s%n",
                s.probeKey, s.probeOk, s.probeFail, s.probeLatencyMs, s.probeAgeMs,
                s.probeError == null ? "" : ", last err " + s.probeError));
        if (s.probeUnsupported) {
            b.append("        no async read answers on this RC: rc_ok uses KeyConnection alone\n");
        }
        b.append(String.format(Locale.US, "        stick read: ok %d fail %d, last %d ms%s%n",
                s.stickProbeOk, s.stickProbeFail, s.stickProbeLatencyMs,
                s.stickProbeError == null ? "" : ", err " + s.stickProbeError));
        RfStatus rf = svc.rf();
        b.append(String.format(Locale.US, "RF      wifi %s  wifi_scan %s  bt %s  ble_scan %s  airplane %s%n",
                rf.wifi, rf.wifiScan, rf.bt, rf.bleScan, rf.airplane));
        if (srv != null) {
            b.append(String.format(Locale.US, "STREAM  :%d  rx %d  tx %d  rejected %d  rate %d Hz%s%n",
                    Protocol.PORT, srv.rxPackets, srv.txPackets, srv.rejected, srv.currentRate,
                    srv.lastError == null ? "" : "  last error: " + srv.lastError));
        }
        b.append("SUBSCRIBERS").append(subs.isEmpty() ? "  none\n" : "\n");
        for (UdpStreamServer.Subscriber sub : subs) {
            b.append(String.format(Locale.US, "  %s  %s  %d Hz  for %d s, seen %.1f s ago%n",
                    sub.addr, sub.client, sub.rate, (now - sub.since) / 1000,
                    (now - sub.lastSeen) / 1000.0));
        }
        b.append("\nKEYS  name          group   callbacks  /s   age    seed  value\n");
        for (RcInputReader.KeyStat k : s.stats) {
            Double r = rates.get(k.name);
            b.append(String.format(Locale.US, "  %-13s %-7s %9d %5s %6s  %-4s  %s%n",
                    k.name, k.group, k.count, r == null ? "-" : String.format(Locale.US, "%.0f", r),
                    k.lastMs == 0 ? "never" : (now - k.lastMs) + "ms", k.seeded ? "yes" : "",
                    k.value));
        }
        b.append("\nANDROID INPUT DEVICES (a gamepad here = a no-MSDK fallback)\n").append(devicesText);
        b.append("\nLAST INPUT EVENTS\n");
        for (String e : events) b.append("  ").append(e).append('\n');
        b.append("\nTHIS RUN, last steps (trace.txt)\n").append(CrashLog.recent(15));
        String err = CrashLog.lastError();
        if (err != null) b.append("last non-fatal error: ").append(err).append('\n');
        b.append(CrashLog.build()).append('\n');
        return b.toString();
    }

    private static String sourceText(RcInputReader.Snapshot s) {
        String slow = s.slowReason();
        return slow == null ? "sticks via GAMEPAD" : "sticks via MSDK, ~10 Hz (" + slow + ")";
    }

    /** The virtual joystick's mode, from its median report gap (GamepadInput.MOVING_GAP_MS). */
    static String gapText(int gapMs) {
        if (gapMs < 0) return "report gap unknown yet";
        return "median gap " + gapMs + " ms (" + (gapMs <= 30 ? "fast mode" : gapMs >= 70 ? "SLOW mode" : "?") + ")";
    }

    private static String gateText(GamepadInput.Reading g) {
        return "game mode " + onOff(g.gameMode) + ", left stick " + onOff(g.motionLeft)
                + ", right stick " + onOff(g.motionRight);
    }

    private static String onOff(Boolean b) {
        return b == null ? "?" : b ? "on" : "OFF";
    }

    private static String wireText(int[] sticks, int[] dials) {
        Integer[] s = new Integer[sticks.length];
        Integer[] d = new Integer[dials.length];
        for (int i = 0; i < s.length; i++) s[i] = sticks[i];
        for (int i = 0; i < d.length; i++) d[i] = dials[i];
        return wireText(s, d);
    }

    private static String wireText(Integer[] sticks, Integer[] dials) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < sticks.length; i++) {
            b.append(Protocol.STICKS[i]).append(' ').append(sticks[i] == null ? "null"
                    : String.format(Locale.US, "%+d", sticks[i])).append(' ');
        }
        for (int i = 0; i < dials.length; i++) {
            b.append(Protocol.DIALS[i]).append(' ').append(dials[i] == null ? "null"
                    : String.format(Locale.US, "%+d", dials[i])).append(' ');
        }
        return b.toString().trim();
    }

    private static String checkText(GamepadInput.Reading g) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < GamepadInput.AXES.length; i++) {
            String v;
            switch (g.check[i]) {
                case GamepadInput.CHECK_OK:
                    v = "ok";
                    break;
                case GamepadInput.CHECK_CENTRE:
                    v = "centre";
                    break;
                case GamepadInput.CHECK_INVERTED:
                    v = "INVERTED";
                    break;
                default:
                    v = "-";
            }
            b.append(GamepadInput.AXES[i]).append(' ').append(v);
            if (g.check[i] != GamepadInput.CHECK_UNTESTED) b.append(" (max diff ").append(g.checkDiff[i]).append(')');
            b.append("  ");
        }
        return b.toString().trim();
    }

    private static String join(List<String> parts) {
        StringBuilder b = new StringBuilder();
        for (String p : parts) {
            if (b.length() > 0) b.append(", ");
            b.append(p);
        }
        return b.toString();
    }

    // --- Android input diagnostics (spike S2/S4) --------------------------------------

    private void refreshDevices() {
        StringBuilder b = new StringBuilder();
        for (int id : InputDevice.getDeviceIds()) {
            InputDevice d = InputDevice.getDevice(id);
            if (d == null) continue;
            int src = d.getSources();
            b.append(String.format(Locale.US, "  #%d %s  src=0x%x%s%s%s%s%n", id, d.getName(), src,
                    has(src, InputDevice.SOURCE_JOYSTICK) ? " JOYSTICK" : "",
                    has(src, InputDevice.SOURCE_GAMEPAD) ? " GAMEPAD" : "",
                    has(src, InputDevice.SOURCE_DPAD) ? " DPAD" : "",
                    has(src, InputDevice.SOURCE_KEYBOARD) ? " KEYBOARD" : ""));
            List<InputDevice.MotionRange> ranges = d.getMotionRanges();
            if (!ranges.isEmpty()) {
                b.append("     axes:");
                for (InputDevice.MotionRange r : ranges) {
                    b.append(' ').append(MotionEvent.axisToString(r.getAxis()).replace("AXIS_", ""));
                }
                b.append('\n');
            }
        }
        devicesText = b.length() == 0 ? "  none\n" : b.toString();
        // Into trace.txt too, when it changes: if MSDK serves no RC keys without an aircraft
        // (REQUEST_HANDLER_NOT_FOUND for component 4 on product 0, 2026-09-25), an Android
        // gamepad exposing the sticks is the no-MSDK fallback, and this is the evidence.
        if (!devicesText.equals(tracedDevices)) {
            tracedDevices = devicesText;
            CrashLog.step("android input devices:\n" + devicesText.replaceAll("(?m)\\s+$", ""));
        }
    }

    private static boolean has(int sources, int source) {
        return (sources & source) == source;
    }

    private void addEvent(String e) {
        events.addFirst(clock.format(new Date()) + " " + e);
        while (events.size() > 12) events.removeLast();
    }

    private static String devName(int deviceId) {
        InputDevice d = InputDevice.getDevice(deviceId);
        return d == null ? String.valueOf(deviceId) : d.getName();
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent e) {
        String action = e.getAction() == KeyEvent.ACTION_DOWN ? "down"
                : e.getAction() == KeyEvent.ACTION_UP ? "up" : "multi";
        addEvent("key " + KeyEvent.keyCodeToString(e.getKeyCode()) + " " + action
                + " dev=" + devName(e.getDeviceId()));
        // D-pad keys from a joystick (Android synthesizes them from unconsumed stick motion)
        // must never move focus around this screen: swallowed.
        if (isFromJoystick(e.getSource()) && isDpad(e.getKeyCode())) return true;
        return super.dispatchKeyEvent(e);
    }

    private static boolean isFromJoystick(int source) {
        return has(source, InputDevice.SOURCE_JOYSTICK) || has(source, InputDevice.SOURCE_GAMEPAD);
    }

    private static boolean isDpad(int k) {
        return k == KeyEvent.KEYCODE_DPAD_UP || k == KeyEvent.KEYCODE_DPAD_DOWN
                || k == KeyEvent.KEYCODE_DPAD_LEFT || k == KeyEvent.KEYCODE_DPAD_RIGHT
                || k == KeyEvent.KEYCODE_DPAD_CENTER;
    }

    /**
     * The RC's gamepad: every report goes to GamepadInput, and the event is CONSUMED, so
     * Android never synthesizes D-pad navigation from it. Another joystick (a USB gamepad
     * plugged into the RC, say) is consumed too, and drives nothing.
     */
    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent e) {
        if (!has(e.getSource(), InputDevice.SOURCE_JOYSTICK)) return super.dispatchGenericMotionEvent(e);
        InputDevice d = e.getDevice();
        int samples = 1 + e.getHistorySize();           // batched samples are reports too
        if (d != null && d.getVendorId() == GAMEPAD_VENDOR && d.getProductId() == GAMEPAD_PRODUCT) {
            gamepadReports += samples;
            long[] times = new long[samples];           // the kernel's timestamps, oldest first
            for (int h = 0; h < samples - 1; h++) times[h] = e.getHistoricalEventTime(h);
            times[samples - 1] = e.getEventTime();
            GamepadInput.INSTANCE.report(e.getAxisValue(MotionEvent.AXIS_X), e.getAxisValue(MotionEvent.AXIS_Y),
                    e.getAxisValue(MotionEvent.AXIS_RX), e.getAxisValue(MotionEvent.AXIS_RY),
                    e.getAxisValue(MotionEvent.AXIS_Z), e.getAxisValue(MotionEvent.AXIS_RZ), times);
        } else {
            foreignReports += samples;
        }
        logMotion(e);
        return true;
    }

    private void logMotion(MotionEvent e) {
        long now = SystemClock.uptimeMillis();
        if (now - lastMotionLog >= 200) {           // at most 5 lines a second
            lastMotionLog = now;
            StringBuilder b = new StringBuilder("motion dev=" + devName(e.getDeviceId())
                    + " src=0x" + Integer.toHexString(e.getSource()));
            InputDevice d = e.getDevice();
            if (d != null) {
                for (InputDevice.MotionRange r : d.getMotionRanges()) {
                    float v = e.getAxisValue(r.getAxis());
                    if (Math.abs(v) > 0.01f) {
                        b.append(' ').append(MotionEvent.axisToString(r.getAxis()).replace("AXIS_", ""))
                                .append('=').append(String.format(Locale.US, "%.2f", v));
                    }
                }
            }
            addEvent(b.toString());
        }
    }

    @Override
    public void onInputDeviceAdded(int deviceId) {
        addEvent("device added: " + devName(deviceId));
        refreshDevices();
    }

    @Override
    public void onInputDeviceRemoved(int deviceId) {
        addEvent("device removed: #" + deviceId);
        refreshDevices();
    }

    @Override
    public void onInputDeviceChanged(int deviceId) {
        refreshDevices();
    }
}
