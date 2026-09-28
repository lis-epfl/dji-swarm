package com.liscontroller;

import android.os.SystemClock;

/**
 * The RC Pro's built-in USB gamepad ("DJI embedded joystick") as the FAST source for the
 * sticks and dials. With MSDK registered it reports ~70 times a second, against ~10
 * changes a second through MSDK's stick keys (measured 2026-09-25). MSDK stays the
 * authority on everything else (rc_ok, the aircraft interlock, the buttons) and is the
 * fallback whenever the gamepad cannot be trusted.
 *
 * Two things make it conditional:
 * - DJI's framework (com.dji.comkey.ComKeyManager, in system_server) drops this device's
 *   events for every non-system app unless the global setting dji_lab_game_mode is 1, and
 *   drops a stick's events unless dji_motion_via_left/right_joystick_enabled is 1.
 *   enable-gamepad.ps1 sets them.
 * - Android delivers joystick events only to the FOCUSED window, so MainActivity must be
 *   in front. It feeds this class, and RcInputReader.snapshot() reads it.
 *
 * The gamepad reports only on change, so its silence cannot tell "held still" from "not
 * delivered". MSDK is the independent witness: a real MSDK stick or dial change that the
 * gamepad did not report around the same time means the gamepad is not delivering (focus
 * lost, game mode switched off, DJI holding the events for a Back-button shortcut), and
 * the sticks fall back to MSDK until the gamepad reports again.
 *
 * It also cross-checks MSDK against the gamepad while both are still, because a sign
 * error on the fallback path would reverse a stick the moment the app lost focus. An axis
 * MSDK reads INVERTED is withheld on the fallback path (RcInputReader nulls it, which the
 * PC treats as no joystick) rather than flown backwards.
 *
 * Pure Java apart from SystemClock and CrashLog, so the JVM wire test can drive it.
 * Thread-safe: the UI thread writes, the stream's tx thread and the UI read.
 */
final class GamepadInput {
    static final GamepadInput INSTANCE = new GamepadInput();

    /** Wire units at full deflection (PROTOCOL.md: raw +-660). */
    static final int FULL = 660;
    /** The sticks' flat as the device reports it (rawFlat 128 of +-32768): reads 0 inside. */
    static final float STICK_FLAT = 0.0039f;
    /** The dials are 8-bit with their centre between two steps: one step either side is 0. */
    static final float DIAL_FLAT = 0.008f;
    /** MainActivity ticks at 10 Hz while resumed; older than this = the screen is not live. */
    static final long UI_FRESH_MS = 500;
    /** MSDK saw real motion this long after the gamepad's last report: the gamepad is silent. */
    static final long WITNESS_GAP_MS = 300;
    /** Both sources unchanged this long = both still, so their values can be compared. */
    static final long STILL_MS = 400;
    /** A deflection big enough for a sign comparison, in wire units (~23%). */
    static final int DEFLECTED = 150;

    // Cross-check verdicts, per axis lh lv rh rv l r.
    static final int CHECK_UNTESTED = 0;
    static final int CHECK_CENTRE = 1;      // agreed at centre only
    static final int CHECK_OK = 2;          // agreed in sign while deflected
    static final int CHECK_INVERTED = 3;    // opposite signs while both deflected: sticky
    static final String[] AXES = {"lh", "lv", "rh", "rv", "l", "r"};

    private static final int MAX_TRACES = 200;
    /**
     * The RC's virtual joystick reports in one of two modes, ~14 ms apart or ~100 ms apart
     * (2026-09-25: both seen with MSDK registered, trigger unknown). Report gaps shorter
     * than this are "while moving" and feed the median that tells the modes apart.
     */
    static final long MOVING_GAP_MS = 400;
    private static final int GAP_RING = 32;

    /** What the stream and the UI see; an immutable copy. */
    static final class Reading {
        boolean usable;
        String whyNot;                      // null when usable
        boolean anyReport;                  // sticks/dials below hold a real report
        final int[] sticks = new int[4];    // wire units, + = up / right
        final int[] dials = new int[2];     // wire units, + = turned right
        int hz;                             // reports in the last second
        int gapMs = -1;                     // median gap between reports while moving, -1 = unknown
        long ageMs = -1;                    // since the last report, -1 = never
        boolean focused;
        Boolean gameMode, motionLeft, motionRight;  // DJI's gate settings, null = unread
        final int[] check = new int[6];     // CHECK_* per axis
        final int[] checkDiff = new int[6]; // largest |gamepad - MSDK| while both still
    }

    // Guarded by `this`.
    private final float[] ax = new float[6];     // X Y RX RY Z RZ, as Android normalizes them
    private long lastReportMs;                   // 0 = never
    private long focusGainedMs;
    private boolean focused;
    private long uiTickMs;
    private int reportsInWindow, hz;
    private long windowStartMs;
    private Boolean gameMode, motionLeft, motionRight;
    private long msdkMotionMs;
    private final int[] check = new int[6];
    private final int[] checkDiff = new int[6];
    private Boolean wasUsable;
    private String wasWhy;
    private int traces;
    private long lastSampleMs;                   // the event time of the last sample
    private final long[] gaps = new long[GAP_RING];
    private int gapCount, gapNext;

    private GamepadInput() {
    }

    // --- writers (UI thread, except msdkMotion) ------------------------------------------

    /**
     * One MotionEvent from the RC's gamepad. `sampleTimesMs` are the event times of its
     * samples, batched history first (MotionEvent.getHistoricalEventTime, then
     * getEventTime): the kernel's timestamps, so their gaps show the device's own rate.
     */
    synchronized void report(float x, float y, float rx, float ry, float z, float rz, long[] sampleTimesMs) {
        long now = SystemClock.elapsedRealtime();
        ax[0] = x;
        ax[1] = y;
        ax[2] = rx;
        ax[3] = ry;
        ax[4] = z;
        ax[5] = rz;
        lastReportMs = now;
        roll(now);
        reportsInWindow += sampleTimesMs.length;
        for (long t : sampleTimesMs) {
            long gap = t - lastSampleMs;
            if (lastSampleMs > 0 && gap > 0 && gap < MOVING_GAP_MS) {
                gaps[gapNext] = gap;
                gapNext = (gapNext + 1) % GAP_RING;
                if (gapCount < GAP_RING) gapCount++;
            }
            lastSampleMs = t;
        }
    }

    synchronized void focus(boolean has) {
        if (has && !focused) focusGainedMs = SystemClock.elapsedRealtime();
        focused = has;
    }

    /** MainActivity's 10 Hz refresh, which runs only while it is resumed. */
    synchronized void uiTick() {
        uiTickMs = SystemClock.elapsedRealtime();
    }

    /** DJI's gate settings, read once a second by the service. null = could not be read. */
    synchronized void gate(Boolean gameMode, Boolean motionLeft, Boolean motionRight) {
        this.gameMode = gameMode;
        this.motionLeft = motionLeft;
        this.motionRight = motionRight;
    }

    /** RcInputReader: MSDK reported a real stick or dial change. */
    synchronized void msdkMotion(long now) {
        msdkMotionMs = now;
    }

    // --- readers ------------------------------------------------------------------------

    Reading read(long now) {
        Reading r = new Reading();
        String trace = null;
        synchronized (this) {
            roll(now);
            String why = whyNot(now);
            r.usable = why == null;
            r.whyNot = why;
            r.anyReport = lastReportMs > 0;
            r.ageMs = lastReportMs > 0 ? now - lastReportMs : -1;
            r.hz = hz;
            r.gapMs = medianGap();
            r.focused = focused;
            r.gameMode = gameMode;
            r.motionLeft = motionLeft;
            r.motionRight = motionRight;
            toWire(r.sticks, r.dials);
            System.arraycopy(check, 0, r.check, 0, check.length);
            System.arraycopy(checkDiff, 0, r.checkDiff, 0, checkDiff.length);
            boolean changed = wasUsable == null || wasUsable != r.usable
                    || (why != null && !why.equals(wasWhy));
            if (changed && traces < MAX_TRACES) {
                traces++;
                trace = r.usable ? "stick source -> GAMEPAD (" + hz + " reports/s)"
                        : "stick source -> MSDK: " + why;
            }
            wasUsable = r.usable;
            wasWhy = why;
        }
        if (trace != null) CrashLog.step(trace);
        return r;
    }

    /**
     * Compare MSDK's values (already in the wire convention) with the gamepad's, for every
     * axis on which both have been unchanged for STILL_MS. Only while the gamepad is
     * usable: a gamepad that stopped delivering holds an old value.
     */
    void crossCheck(Integer[] msdkSticks, Integer[] msdkDials, long[] msdkChangeMs, long now) {
        StringBuilder trace = null;
        synchronized (this) {
            if (whyNot(now) != null || now - lastReportMs < STILL_MS) return;
            int[] gs = new int[4];
            int[] gd = new int[2];
            toWire(gs, gd);
            for (int i = 0; i < 6; i++) {
                Integer m = i < 4 ? msdkSticks[i] : msdkDials[i - 4];
                if (m == null || now - msdkChangeMs[i] < STILL_MS) continue;
                int g = i < 4 ? gs[i] : gd[i - 4];
                checkDiff[i] = Math.max(checkDiff[i], Math.abs(g - m));
                int before = check[i];
                if (before == CHECK_INVERTED) continue;
                boolean bothDeflected = Math.abs(g) >= DEFLECTED && Math.abs(m) >= DEFLECTED;
                if (bothDeflected && Integer.signum(g) != Integer.signum(m)) {
                    check[i] = CHECK_INVERTED;
                } else if (bothDeflected) {
                    check[i] = CHECK_OK;
                } else if (before == CHECK_UNTESTED && Math.abs(g) < DEFLECTED && Math.abs(m) < DEFLECTED) {
                    check[i] = CHECK_CENTRE;
                }
                if (check[i] != before && check[i] != CHECK_CENTRE && traces < MAX_TRACES) {
                    traces++;
                    if (trace == null) trace = new StringBuilder();
                    trace.append(check[i] == CHECK_INVERTED
                            ? "CROSS-CHECK: MSDK reads " + AXES[i] + " INVERTED vs the gamepad (gamepad "
                                    + g + ", MSDK " + m + ") - the MSDK fallback withholds " + AXES[i]
                                    + "; fix its sign in RcInputReader"
                            : "cross-check: " + AXES[i] + " agrees (gamepad " + g + ", MSDK " + m + ")")
                            .append('\n');
                }
            }
        }
        if (trace != null) CrashLog.step(trace.toString().trim());
    }

    // --- internals (under `this`) ---------------------------------------------------------

    private String whyNot(long now) {
        if (Boolean.FALSE.equals(gameMode)) return "DJI game mode is off (enable-gamepad.ps1)";
        if (Boolean.FALSE.equals(motionLeft) || Boolean.FALSE.equals(motionRight)) {
            return "DJI stick motion is off (enable-gamepad.ps1)";
        }
        if (!focused) return "the app is not in front";
        if (uiTickMs == 0 || now - uiTickMs > UI_FRESH_MS) return "the screen is not updating";
        if (lastReportMs == 0 || lastReportMs < focusGainedMs) return "no gamepad report yet";
        if (msdkMotionMs - lastReportMs > WITNESS_GAP_MS) {
            return "the gamepad is silent while MSDK sees the controls move";
        }
        return null;
    }

    private void toWire(int[] sticks, int[] dials) {
        sticks[0] = wire(ax[0], STICK_FLAT);
        sticks[1] = wire(-ax[1], STICK_FLAT);       // the HID's up is -1; the wire's up is +
        sticks[2] = wire(ax[2], STICK_FLAT);
        sticks[3] = wire(-ax[3], STICK_FLAT);
        dials[0] = wire(ax[4], DIAL_FLAT);          // turned right = + on both
        dials[1] = wire(ax[5], DIAL_FLAT);
    }

    /** Median of the recent moving gaps, ms; -1 until 5 exist. */
    private int medianGap() {
        if (gapCount < 5) return -1;
        long[] g = new long[gapCount];
        System.arraycopy(gaps, 0, g, 0, gapCount);
        java.util.Arrays.sort(g);
        return (int) g[gapCount / 2];
    }

    static int wire(float v, float flat) {
        if (Float.isNaN(v) || Math.abs(v) < flat) return 0;
        return Math.max(-FULL, Math.min(FULL, Math.round(v * FULL)));
    }

    private void roll(long now) {
        if (windowStartMs == 0) windowStartMs = now;
        long d = now - windowStartMs;
        if (d >= 1000) {
            hz = (int) (reportsInWindow * 1000L / d);
            reportsInWindow = 0;
            windowStartMs = now;
        }
    }
}
