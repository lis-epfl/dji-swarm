package at.jku.icg.aos_dji_sdkv5.core;

import android.util.Log;
import io.moquette.broker.Server;
import io.moquette.broker.config.IConfig;
import io.moquette.broker.config.MemoryConfig;
import io.moquette.interception.AbstractInterceptHandler;
import io.moquette.interception.messages.InterceptPublishMessage;
import io.netty.buffer.Unpooled;
import io.netty.handler.codec.mqtt.MqttMessageBuilders;
import io.netty.handler.codec.mqtt.MqttPublishMessage;
import io.netty.handler.codec.mqtt.MqttQoS;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.Properties;

public final class MQTTEmbedded {
    public static final String TAG = "MQTTEmbedded";

    /**
     * Topic the app publishes its own read-only link diagnostics on (the PC
     * subscribes; see SwarmActivity.publishLinkStatus/publishLinkScan).
     * Deliberately NOT the command topic: the broker routes by topic for real
     * subscribers, so keeping diagnostics off MQTTWayPoints stops them landing
     * in the PC's identity-probe capture buffer.
     */
    public static final String DIAG_TOPIC = "LISSwarmDiag";

    /** Client id attributed to broker-internal publishes. */
    private static final String DIAG_CLIENT_ID = "LIS_APP_DIAG";

    private static AOSManager aosManager;
    private static String dataDir;
    public static Server mqttBroker;

    /** Listener for incoming MQTT commands (joystick or waypoint). */
    public interface CommandListener {
        void onCommandReceived(String command);
    }

    private static CommandListener commandListener;

    public void setCommandListener(CommandListener listener) {
        commandListener = listener;
    }

    static class PublisherListener extends AbstractInterceptHandler {
        @Override
        public String getID() {
            return "MQTTEmbeddedPublishListener";
        }

        PublisherListener() {
        }

        @Override
        public void onPublish(InterceptPublishMessage interceptPublishMessage) {
            String payload = interceptPublishMessage.getPayload().toString(StandardCharsets.UTF_8);
            if (MQTTEmbedded.commandListener != null) {
                MQTTEmbedded.commandListener.onCommandReceived(payload);
            }
        }

        public void onSessionLoopError(Throwable th) {
            Log.e(TAG, "Session event loop reported error: " + th);
        }
    }

    public void run() throws InterruptedException, IOException {
        Properties props = new Properties();
        props.setProperty("port", "1883");
        props.setProperty("host", "0.0.0.0");
        props.setProperty("allow_anonymous", "true");
        if (dataDir != null) {
            props.setProperty("persistent_store", dataDir + "/moquette_store.h2");
        }
        IConfig config = new MemoryConfig(props);
        Server server = new Server();
        mqttBroker = server;
        server.startServer(config, Collections.singletonList(new PublisherListener()));
        Log.v(TAG, "MQTT moquette Broker started on 0.0.0.0:1883");
    }

    /**
     * Publish a message from inside the app onto its own embedded broker, so
     * the PC's persistent per-RC connection can subscribe to it. This is the
     * only app->PC channel that is not the RTSP stream: the telemetry string
     * rides RTSP through setTelemetryData(), whose native signature is fixed
     * (17 fields, no source available), so anything new has to go over MQTT.
     *
     * QoS 0 and NOT retained. Both are deliberate:
     *  - QoS 0 because every payload is a snapshot (latest-wins) and a drop is
     *    corrected by the next one.
     *  - not retained because this broker is configured with a
     *    persistent_store, so a retained 1 Hz status would put an H2 write on
     *    the RC every second; and for the on-demand scan a retained copy is a
     *    hazard, not a feature — a PC that reconnects mid-request would be
     *    handed the PREVIOUS scan and take it for the answer to the new one.
     *    The PC always asks explicitly (LINKDIAG), so it never needs a
     *    subscribe-time replay.
     *
     * Best-effort — never throws, so a broker hiccup cannot take down the
     * caller's timer thread.
     *
     * NOTE: the PublisherListener intercept below fires for internal publishes
     * too, so whatever is published here is also handed to the command
     * listener — SwarmActivity.onCommandReceived filters its own diagnostics
     * back out by prefix.
     */
    public static void publishDiagnostic(String payload) {
        Server broker = mqttBroker;
        if (broker == null || payload == null) {
            return;
        }
        try {
            MqttPublishMessage msg = MqttMessageBuilders.publish()
                .topicName(DIAG_TOPIC)
                .retained(false)
                .qos(MqttQoS.AT_MOST_ONCE)
                .payload(Unpooled.copiedBuffer(payload, StandardCharsets.UTF_8))
                .build();
            broker.internalPublish(msg, DIAG_CLIENT_ID);
        } catch (Exception e) {
            Log.w(TAG, "diagnostic publish failed: " + e);
        }
    }

    public void stop() {
        if (mqttBroker != null) {
            mqttBroker.stopServer();
        }
    }

    public MQTTEmbedded(AOSManager aOSManager, String appDataDir) {
        aosManager = aOSManager;
        dataDir = appDataDir;
    }
}
