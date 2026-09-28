package com.liscontroller;

import android.os.SystemClock;
import android.util.Log;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The RC side of wire protocol v1 (rc-joystick/PROTOCOL.md). The RC is the server:
 * PCs subscribe to UDP :5070, and each gets `state` at the highest rate any live
 * subscriber asked for, plus an `info` reply to every subscribe. Nothing is sent
 * while nobody listens.
 *
 * One wildcard socket. With Wi-Fi off, Ethernet is the only network, and
 * lis-swarm-app serves over eth0 the same way. Binding to the Ethernet Network
 * object is left for the case where the hardware spike shows replies misrouted.
 */
final class UdpStreamServer {
    private static final String TAG = RcInputReader.TAG;
    private static final Charset UTF8 = Charset.forName("UTF-8");

    interface InfoSource {
        RfStatus rf();

        String ethIp();

        String appVersion();
    }

    static final class Subscriber {
        final InetSocketAddress addr;
        final long since;
        volatile String client = "?";
        volatile int rate = Protocol.RATE_DEFAULT_HZ;
        volatile long lastSeen;

        Subscriber(InetSocketAddress addr, long now) {
            this.addr = addr;
            this.since = now;
            this.lastSeen = now;
        }
    }

    private final RcInputReader reader;
    private final InfoSource info;
    private final Map<InetSocketAddress, Subscriber> subs = new ConcurrentHashMap<>();
    private final Object txLock = new Object();
    private boolean wakeFlag;                       // guarded by txLock
    private volatile boolean running;
    private DatagramSocket socket;
    private Thread rx, tx;
    private long seq;                               // tx thread only

    volatile int currentRate;
    volatile long rxPackets, txPackets, rejected;
    volatile String lastError;

    UdpStreamServer(RcInputReader reader, InfoSource info) {
        this.reader = reader;
        this.info = info;
    }

    void start() throws SocketException {
        DatagramSocket s = new DatagramSocket(null);
        s.setReuseAddress(true);
        s.bind(new InetSocketAddress(Protocol.PORT));
        s.setSoTimeout(500);
        socket = s;
        running = true;
        rx = new Thread(this::rxLoop, "lisc-rx");
        tx = new Thread(this::txLoop, "lisc-tx");
        rx.start();
        tx.start();
        Log.i(TAG, "stream: listening on UDP :" + Protocol.PORT);
    }

    /** Ask the tx thread to send now (a button edge). Any thread. */
    void wake() {
        synchronized (txLock) {
            wakeFlag = true;
            txLock.notifyAll();
        }
    }

    /**
     * Stop, telling every subscriber "bye" so the PC goes stale at once instead of
     * timing out. Safe on the main thread: all network I/O runs on a worker.
     */
    void stop(final String reason) {
        if (!running) return;
        running = false;
        wake();
        Thread t = new Thread(() -> {
            join(tx, 300);                          // no state may follow the bye
            DatagramSocket s = socket;
            try {
                byte[] bye = Protocol.bye(reason).toString().getBytes(UTF8);
                for (Subscriber sub : subs.values()) {
                    try {
                        s.send(new DatagramPacket(bye, bye.length, sub.addr));
                    } catch (IOException ignored) {
                    }
                }
            } catch (JSONException ignored) {
            }
            subs.clear();
            s.close();                              // unblocks rx
            join(rx, 700);
        }, "lisc-stop");
        t.start();
        join(t, 1500);
        Log.i(TAG, "stream: stopped (" + reason + ")");
    }

    List<Subscriber> subscribers() {
        return new ArrayList<>(subs.values());
    }

    boolean isRunning() {
        return running;
    }

    // --- rx: subscriptions and info replies ----------------------------------------

    private void rxLoop() {
        byte[] buf = new byte[4096];
        while (running) {
            DatagramPacket p = new DatagramPacket(buf, buf.length);
            try {
                socket.receive(p);
            } catch (SocketTimeoutException e) {
                expire();
                continue;
            } catch (IOException e) {
                if (!running) break;
                lastError = "rx: " + e.getMessage();
                SystemClock.sleep(200);
                continue;
            }
            rxPackets++;
            handle(p);
        }
    }

    private void handle(DatagramPacket p) {
        JSONObject m;
        try {
            m = new JSONObject(new String(p.getData(), p.getOffset(), p.getLength(), UTF8));
        } catch (JSONException e) {
            rejected++;
            return;
        }
        Object v = m.opt("v");
        if (!(v instanceof Integer) || (Integer) v != Protocol.VERSION) {
            rejected++;                             // another protocol version: never guess
            return;
        }
        InetSocketAddress from = (InetSocketAddress) p.getSocketAddress();
        long now = SystemClock.elapsedRealtime();
        String type = m.optString("type", "");
        if ("subscribe".equals(type)) {
            Subscriber s = subs.get(from);
            boolean isNew = s == null;
            if (isNew) {
                s = new Subscriber(from, now);
                subs.put(from, s);
            }
            s.client = m.optString("client", "?");
            s.rate = Protocol.clampRate(m.optInt("rate_hz", Protocol.RATE_DEFAULT_HZ));
            s.lastSeen = now;
            if (isNew) Log.i(TAG, "stream: + " + from + " (" + s.client + ", " + s.rate + " Hz)");
            Object t = m.opt("t");
            sendInfo(from, t instanceof Number ? t : null);
            if (isNew) wake();                      // start streaming to it at once
        } else if ("unsubscribe".equals(type)) {
            Subscriber gone = subs.remove(from);
            if (gone != null) Log.i(TAG, "stream: - " + from + " (" + gone.client + ")");
        }
    }

    private void sendInfo(InetSocketAddress to, Object echoT) {
        try {
            RcInputReader.Snapshot s = reader.snapshot(true);
            RfStatus rf = info.rf();
            byte[] b = Protocol.info(echoT, s, rf, info.ethIp(), info.appVersion(),
                    currentRate, subs.size(), true).toString().getBytes(UTF8);
            if (b.length > Protocol.MAX_INFO_BYTES) {
                // Must stay one unfragmented datagram: drop the per-key table first.
                Log.w(TAG, "stream: info is " + b.length + " B, sending it without keys");
                b = Protocol.info(echoT, s, rf, info.ethIp(), info.appVersion(),
                        currentRate, subs.size(), false).toString().getBytes(UTF8);
            }
            socket.send(new DatagramPacket(b, b.length, to));
        } catch (JSONException | IOException e) {
            lastError = "info: " + e.getMessage();
        }
    }

    private void expire() {
        long now = SystemClock.elapsedRealtime();
        for (Iterator<Subscriber> it = subs.values().iterator(); it.hasNext(); ) {
            Subscriber s = it.next();
            if (now - s.lastSeen > Protocol.SUBSCRIPTION_TTL_MS) {
                it.remove();
                Log.i(TAG, "stream: x " + s.addr + " (" + s.client + ") expired");
            }
        }
    }

    // --- tx: the state stream -------------------------------------------------------

    private void txLoop() {
        long next = SystemClock.elapsedRealtimeNanos();
        while (running) {
            expire();
            if (subs.isEmpty()) {
                currentRate = 0;
                waitNanos(100_000_000L);
                next = SystemClock.elapsedRealtimeNanos();
                continue;
            }
            int rate = Protocol.RATE_MIN_HZ;
            for (Subscriber s : subs.values()) rate = Math.max(rate, s.rate);
            currentRate = rate;
            sendState();
            next += 1_000_000_000L / rate;
            long now = SystemClock.elapsedRealtimeNanos();
            if (next - now <= 0) {
                next = now;                         // fell behind: resync, don't burst
            } else if (waitNanos(next - now)) {
                next = SystemClock.elapsedRealtimeNanos();  // a press: send now
            }
        }
    }

    private void sendState() {
        byte[] b;
        try {
            b = Protocol.state(seq, reader.snapshot(false)).toString().getBytes(UTF8);
        } catch (JSONException e) {
            lastError = "state: " + e.getMessage();
            return;
        }
        for (Subscriber s : subs.values()) {
            try {
                socket.send(new DatagramPacket(b, b.length, s.addr));
                txPackets++;
            } catch (IOException e) {
                lastError = "tx " + s.addr + ": " + e.getMessage();
            }
        }
        seq++;
    }

    /** Wait up to `nanos`; true if wake() cut it short. */
    private boolean waitNanos(long nanos) {
        synchronized (txLock) {
            if (!wakeFlag && running) {
                try {
                    txLock.wait(nanos / 1_000_000L, (int) (nanos % 1_000_000L));
                } catch (InterruptedException e) {
                    return false;
                }
            }
            boolean woke = wakeFlag;
            wakeFlag = false;
            return woke;
        }
    }

    private static void join(Thread t, long ms) {
        if (t == null) return;
        try {
            t.join(ms);
        } catch (InterruptedException ignored) {
        }
    }
}
