package com.liscontroller;

import android.content.Context;
import android.content.pm.PackageInfo;
import android.os.Build;
import android.os.Process;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Crash capture and a startup trace. It is installed first thing in attachBaseContext,
 * BEFORE DJI's Helper.install(), so it sees a crash anywhere in the process.
 *
 *   trace.txt       one line per startup/stage step, written and flushed as it happens,
 *                   so it survives even a NATIVE crash, which no Java handler catches.
 *                   Rotated to trace.prev.txt at every start, so the run that died is kept.
 *   crash.txt       an uncaught Java exception: stack trace, the last steps, the device
 *                   and the build. Its presence starts the next launch in SAFE MODE.
 *
 * Both go to the app's internal files dir (adb shell run-as com.liscontroller cat
 * files/crash.txt), and to its external files dir, Android/data/com.liscontroller/files/,
 * which a PC reaches over USB with adb pull or a file browser: no root, no run-as.
 * rc-joystick/android/collect-debug.ps1 gathers all of it.
 */
final class CrashLog {
    static final String TAG = "LIS_CONTROLLER";
    static final String CRASH = "crash.txt";
    static final String TRACE = "trace.txt";
    static final String TRACE_PREV = "trace.prev.txt";

    private static final Charset UTF8 = Charset.forName("UTF-8");
    private static final int RING = 150;
    private static final Object lock = new Object();
    private static final ArrayDeque<String> ring = new ArrayDeque<>();
    private static File[] dirs = new File[0];
    private static FileOutputStream[] traces = new FileOutputStream[0];
    private static String build = "?";
    private static String lastError;
    private static boolean installed;
    private static boolean crashWritten;

    private CrashLog() {
    }

    /** Call once, first thing in Application.attachBaseContext. */
    static void install(Context ctx) {
        synchronized (lock) {
            if (installed) return;
            installed = true;
            List<File> ds = new ArrayList<>();
            ds.add(ctx.getFilesDir());
            File ext = null;
            try {
                ext = ctx.getExternalFilesDir(null);
            } catch (RuntimeException ignored) {
            }
            if (ext != null) ds.add(ext);
            dirs = ds.toArray(new File[0]);
            traces = new FileOutputStream[dirs.length];
            for (int i = 0; i < dirs.length; i++) {
                File cur = new File(dirs[i], TRACE);
                File prev = new File(dirs[i], TRACE_PREV);
                if (cur.exists()) {
                    //noinspection ResultOfMethodCallIgnored
                    prev.delete();
                    //noinspection ResultOfMethodCallIgnored
                    cur.renameTo(prev);
                }
                try {
                    traces[i] = new FileOutputStream(cur, false);
                } catch (IOException e) {
                    traces[i] = null;
                }
            }
            build = describeBuild(ctx);
        }
        Thread.setDefaultUncaughtExceptionHandler(
                new Catcher(Thread.getDefaultUncaughtExceptionHandler()));
        step("process start, pid " + Process.myPid() + " | " + build);
    }

    /**
     * MSDK / secneo may install their own default handler after ours without chaining
     * to it, which would silently stop crash.txt from being written for any crash after
     * SDK start. Called after each of those steps: if ours is no longer the default, it
     * goes back on top and delegates to whatever is there now.
     */
    static void reassert(String when) {
        Thread.UncaughtExceptionHandler cur = Thread.getDefaultUncaughtExceptionHandler();
        if (cur instanceof Catcher) return;
        Thread.setDefaultUncaughtExceptionHandler(new Catcher(cur));
        step("crash handler put back on top of " + (cur == null ? "none" : cur.getClass().getName())
                + " (" + when + ")");
    }

    /** Ours: write crash.txt, then hand over (Android's logs it and shows the dialog). */
    private static final class Catcher implements Thread.UncaughtExceptionHandler {
        private final Thread.UncaughtExceptionHandler next;

        Catcher(Thread.UncaughtExceptionHandler next) {
            this.next = next;
        }

        @Override
        public void uncaughtException(Thread thread, Throwable e) {
            try {
                writeCrash(thread, e);
            } catch (Throwable ignored) {
            }
            if (next != null) {
                next.uncaughtException(thread, e);
            } else {
                Process.killProcess(Process.myPid());
                System.exit(10);
            }
        }
    }

    /** One startup/stage step: logcat, the in-memory ring and trace.txt (flushed). */
    static void step(String what) {
        String line = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date())
                + " [" + Thread.currentThread().getName() + "] " + what;
        Log.i(TAG, "STEP " + what);
        synchronized (lock) {
            ring.addLast(line);
            while (ring.size() > RING) ring.removeFirst();
            byte[] b = (line + "\n").getBytes(UTF8);
            for (FileOutputStream f : traces) {
                if (f == null) continue;
                try {
                    f.write(b);
                    f.flush();
                } catch (IOException ignored) {
                }
            }
        }
    }

    /** A non-fatal failure: logged with its stack, traced, and kept for the screen. */
    static void error(String where, Throwable e) {
        Log.e(TAG, where, e);
        step("ERROR " + where + ": " + e);
        synchronized (lock) {
            lastError = where + ": " + e;
        }
    }

    static String lastError() {
        synchronized (lock) {
            return lastError;
        }
    }

    /** The last n steps of this run, oldest first. */
    static String recent(int n) {
        StringBuilder b = new StringBuilder();
        synchronized (lock) {
            int skip = Math.max(0, ring.size() - n), i = 0;
            for (String s : ring) {
                if (i++ >= skip) b.append("  ").append(s).append('\n');
            }
        }
        return b.toString();
    }

    static String build() {
        synchronized (lock) {
            return build;
        }
    }

    static boolean hasCrash(Context ctx) {
        return new File(ctx.getFilesDir(), CRASH).exists();
    }

    static String readCrash(Context ctx) {
        return read(new File(ctx.getFilesDir(), CRASH));
    }

    static String readPreviousTrace(Context ctx) {
        return read(new File(ctx.getFilesDir(), TRACE_PREV));
    }

    static void clearCrash(Context ctx) {
        //noinspection ResultOfMethodCallIgnored
        new File(ctx.getFilesDir(), CRASH).delete();
        synchronized (lock) {
            for (File d : dirs) {
                //noinspection ResultOfMethodCallIgnored
                new File(d, CRASH).delete();
            }
        }
        step("crash report cleared");
    }

    // --- internals -------------------------------------------------------------------

    private static void writeCrash(Thread thread, Throwable e) {
        synchronized (lock) {
            if (crashWritten) return;       // a chained second Catcher: already on disk
            crashWritten = true;
        }
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        pw.println("LIS_CONTROLLER CRASH  "
                + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()));
        pw.println(build());
        pw.println("thread: " + thread.getName());
        pw.println();
        e.printStackTrace(pw);
        pw.println();
        pw.println("last steps before the crash:");
        pw.print(recent(RING));
        pw.flush();
        String text = sw.toString();
        Log.e(TAG, "FATAL, report written to " + CRASH + "\n" + text);
        byte[] b = text.getBytes(UTF8);
        File[] ds;
        synchronized (lock) {
            ds = dirs;
        }
        for (File d : ds) {
            try (FileOutputStream f = new FileOutputStream(new File(d, CRASH), false)) {
                f.write(b);
            } catch (IOException ignored) {
            }
        }
        step("CRASH " + e);
    }

    private static String describeBuild(Context ctx) {
        String v = "?";
        try {
            PackageInfo pi = ctx.getPackageManager().getPackageInfo(ctx.getPackageName(), 0);
            //noinspection deprecation
            v = pi.versionName + " (" + pi.versionCode + "), installed "
                    + new SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US).format(new Date(pi.lastUpdateTime));
        } catch (Exception ignored) {
        }
        return "LIS_CONTROLLER " + v + " | " + Build.MANUFACTURER + " " + Build.MODEL
                + " | Android " + Build.VERSION.RELEASE + " (API " + Build.VERSION.SDK_INT + ")"
                + " | ABIs " + Arrays.toString(Build.SUPPORTED_ABIS);
    }

    private static String read(File f) {
        if (!f.exists()) return null;
        try (FileInputStream in = new FileInputStream(f)) {
            byte[] b = new byte[(int) Math.min(f.length(), 64 * 1024)];
            int n = 0, r;
            while (n < b.length && (r = in.read(b, n, b.length - n)) > 0) n += r;
            return new String(b, 0, n, UTF8);
        } catch (IOException e) {
            return "(could not read " + f + ": " + e + ")";
        }
    }
}
