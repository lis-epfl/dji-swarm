"""
LIS_Swarm direct MQTT command sender
====================================
Publishes command strings (VS:..., ENABLE_VS, TAKEOFF, ...) straight to each
RC's embedded Moquette broker (tcp://<rc-ip>:1883) over ONE persistent
connection per drone.

This bypasses DroneSwarmServer's send path, which builds and tears down a full
MQTT connection per command (Dialog1Dlg.cpp SendWayPoint2Drone2: create ->
TCP connect -> publish -> waitForCompletion -> disconnect x2 -> destroy,
~220 ms each). That capped commands at ~4.5 Hz with one drone and ~1.5 Hz per
drone with three. A persistent connection pays the connect cost once; each
publish is then a sub-millisecond socket write, so 20 Hz per drone is trivial.

The app side needs no changes: its Moquette intercept handler fires
onCommandReceived for ANY publish from ANY client regardless of topic
(MQTTEmbedded.java), and the payload is the same raw command string.

Design notes:
- paho-mqtt 2.x API (CallbackAPIVersion.VERSION2), already installed for the
  project's Python 3.7.
- One client + network thread per drone via loop_start(); connect_async +
  reconnect_delay_set give automatic reconnection, so an RC power-cycle or
  cable pull self-heals.
- Unique client ids (LIS_PC_d<N>_<pid>) so connections can't kick each other
  or the server's transient "MQTTDroneSwarm" client off the broker.
- QoS policy (chosen by the caller per send): QoS 0 for the 20 Hz VS stream
  (latest-wins; drops while disconnected are correct), QoS 1 for one-shot
  commands (ENABLE_VS/DISABLE_VS/TAKEOFF/LAND) — paho queues QoS 1 publishes
  while disconnected and delivers them on reconnect.
- Best-effort like the other feed publishers: send() never raises and never
  blocks the control loops; failures print throttled warnings.

No ds_wrapper import — telemetry/video still flow through DroneSwarmServer.
"""

import os
import threading
import time

import paho.mqtt.client as mqtt

# Topic the server's send path also uses (Dialog1Dlg.cpp TOPIC). The app's
# intercept handler ignores the topic, but keeping it consistent aids sniffing.
COMMAND_TOPIC = "MQTTWayPoints"
BROKER_PORT = 1883
KEEPALIVE_S = 10

# Minimum seconds between repeated "publish failed" warnings per drone.
_WARN_INTERVAL_S = 1.0

# Cap on messages buffered during an identity-probe capture window. The 20 Hz
# VS streams also publish on COMMAND_TOPIC and echo back to subscribers, so a
# few seconds of probing can accumulate a few hundred rows; the cap only guards
# against a capture window accidentally left open.
_PROBE_BUFFER_MAX = 5000


class MqttCommandSender:
    """Persistent per-drone MQTT publishers for drone command strings."""

    def __init__(self, ip_by_drone, port=BROKER_PORT, topic=COMMAND_TOPIC,
                 keepalive=KEEPALIVE_S):
        """
        Args:
            ip_by_drone: {drone_id (1-based int): "a.b.c.d"} RC broker IPs.
        """
        self._topic = topic
        self._ips = dict(ip_by_drone)
        self._clients = {}
        self._connected = {did: False for did in self._ips}
        self._last_warn = {did: 0.0 for did in self._ips}
        self._stopping = False

        # Identity-probe capture: while _probe_msgs is a list, every message
        # received on any per-IP connection is appended as (drone_id, payload).
        # None = capture off (the steady state; on_message then does nothing).
        self._probe_lock = threading.Lock()
        self._probe_msgs = None

        for did, ip in self._ips.items():
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id="LIS_PC_d{}_{}".format(did, os.getpid()))
            client.on_connect = self._make_on_connect(did)
            client.on_disconnect = self._make_on_disconnect(did)
            client.on_message = self._make_on_message(did)
            client.reconnect_delay_set(min_delay=0.5, max_delay=2)
            # connect_async + loop_start: the network thread owns connecting,
            # keepalive pings, and reconnection; never blocks this thread.
            client.connect_async(ip, port, keepalive)
            client.loop_start()
            self._clients[did] = client

    def _make_on_connect(self, drone_id):
        def on_connect(client, userdata, flags, reason_code, properties):
            self._connected[drone_id] = True
            print("[mqtt-cmd {}] connected to {} ({})".format(
                drone_id, self._ips[drone_id], reason_code), flush=True)
        return on_connect

    def _make_on_disconnect(self, drone_id):
        def on_disconnect(client, userdata, flags, reason_code, properties):
            self._connected[drone_id] = False
            if not self._stopping:
                print("[mqtt-cmd {}] disconnected from {} ({}); "
                      "auto-reconnecting".format(
                          drone_id, self._ips[drone_id], reason_code),
                      flush=True)
        return on_disconnect

    def _make_on_message(self, drone_id):
        def on_message(client, userdata, msg):
            with self._probe_lock:
                if (self._probe_msgs is not None
                        and len(self._probe_msgs) < _PROBE_BUFFER_MAX):
                    self._probe_msgs.append(
                        (drone_id, msg.payload.decode("utf-8", "replace")))
        return on_message

    def connected(self, drone_id):
        return self._connected.get(drone_id, False)

    def ip_of(self, drone_id):
        return self._ips.get(drone_id)

    # ------------------------------------------------------------------ #
    # Identity probe (see swarm_flocking.run_identity_check)
    #
    # DroneSwarmServer's fallback send path publishes to slot N's RC broker
    # (tcp://IPperDrone[N-1]:1883, topic MQTTWayPoints) — the only place the
    # server's slot->IP table is observable without rebuilding it. These
    # helpers let the caller subscribe every per-IP connection to that topic,
    # inject a marker per slot through the server, and see which IP-keyed
    # connection it lands on. Markers are inert on the app side: SwarmActivity
    # .onCommandReceived ignores unknown command strings.
    # ------------------------------------------------------------------ #

    def probe_start(self):
        """Begin capturing incoming messages on all connected clients.

        Returns the list of drone_ids whose connection could NOT subscribe
        (offline RCs) — their IPs are invisible to the probe.
        """
        with self._probe_lock:
            self._probe_msgs = []
        failed = []
        for did, client in self._clients.items():
            ok = False
            if self._connected.get(did):
                try:
                    rc, _mid = client.subscribe(self._topic, qos=1)
                    ok = (rc == mqtt.MQTT_ERR_SUCCESS)
                except Exception:
                    ok = False
            if not ok:
                failed.append(did)
        return failed

    def probe_wait_for(self, marker, timeout=2.0, grace=0.3):
        """Wait for `marker` to appear; return the drone_ids that received it.

        Polls the capture buffer until the marker shows up on at least one
        connection, then keeps watching a short `grace` longer so a duplicate
        delivery (two slots pointing at one RC) is not missed. Empty list on
        timeout.
        """
        deadline = time.monotonic() + timeout
        grace_end = None
        while True:
            with self._probe_lock:
                msgs = list(self._probe_msgs or [])
            hits = sorted({did for did, payload in msgs if payload == marker})
            now = time.monotonic()
            if hits and grace_end is None:
                grace_end = now + grace
            if grace_end is not None and now >= grace_end:
                return hits
            if now >= deadline:
                return hits
            time.sleep(0.05)

    def probe_stop(self):
        """Stop capturing and unsubscribe all connections."""
        with self._probe_lock:
            self._probe_msgs = None
        for did, client in self._clients.items():
            if self._connected.get(did):
                try:
                    client.unsubscribe(self._topic)
                except Exception:
                    pass

    def send(self, drone_id, command, qos=0):
        """Publish a command string to drone_id's broker. Never raises/blocks.

        Returns True if the publish was accepted locally (QoS 0: written to
        the socket buffer; QoS 1: queued for delivery), False otherwise.
        """
        client = self._clients.get(drone_id)
        if client is None:
            self._warn(drone_id, "no MQTT client for drone {}".format(drone_id))
            return False
        try:
            info = client.publish(self._topic, command, qos=qos)
            if info.rc == mqtt.MQTT_ERR_SUCCESS:
                return True
            # QoS 0 while disconnected lands here (MQTT_ERR_NO_CONN): the VS
            # stream is latest-wins, so dropping is the right behaviour.
            self._warn(drone_id, "publish rc={} (dropped: {!r})".format(
                info.rc, command))
        except Exception as e:
            self._warn(drone_id, "publish error: {}".format(e))
        return False

    def _warn(self, drone_id, msg):
        now = time.monotonic()
        if now - self._last_warn.get(drone_id, 0.0) >= _WARN_INTERVAL_S:
            self._last_warn[drone_id] = now
            print("[mqtt-cmd {}] {}".format(drone_id, msg), flush=True)

    def stop(self):
        self._stopping = True
        for did, client in self._clients.items():
            try:
                client.disconnect()
                client.loop_stop()
            except Exception:
                pass
        self._clients = {}
