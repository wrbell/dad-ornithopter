#!/usr/bin/env python3
"""
MCO-1D Tactical Cursor-on-Target (CoT) Bridge
=============================================
Provides seamless bidirectional integration between the MCO-1D Flight Computer
and Department of Defense tactical systems, including:
  - ATAK (Android Tactical Assault Kit)
  - WinTAK / CivTAK / TAK Server
  - MIL-STD Cursor-on-Target (CoT) standard feeds

Outbound:
  Converts MCO-1D telemetry into MIL-STD-2525D friendly military drone CoT events:
    Type: 'a-f-A-M-F-Q' (Atom -> Friend -> Airborne -> Military -> Fixed-Wing -> Drone)
  Broadcasts over UDP Multicast (239.2.3.1:6969) or unicast TAK endpoints.

Inbound:
  Listens for ATAK touch-to-fly waypoints, Sensor Points of Interest (SPI),
  and emergency abort/tasking commands, injecting them into the MCO-1D Autonomy Layer.

Author: MCO-1D Defense Modernization Initiative (September 2026)
"""

import os
import sys
import time
import math
import socket
import struct
import threading
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

# Standard ATAK CoT Multicast Defaults
DEFAULT_COT_MULTICAST_GROUP = "239.2.3.1"
DEFAULT_COT_MULTICAST_PORT  = 6969
DEFAULT_COT_UNICAST_PORT    = 4242
DEFAULT_COT_TYPE            = "a-f-A-M-F-Q"   # Friendly Military Fixed-Wing Drone
DEFAULT_COT_CALLSIGN        = "MCO-1D-01"
DEFAULT_COT_UID             = "MCO-1D-DRONE-01"


def iso_zulu(ts=None):
    """Format Unix timestamp as ISO 8601 UTC Zulu string (e.g. 2026-09-17T11:45:00.000Z)."""
    if ts is None:
        ts = time.time()
    dt = datetime.fromtimestamp(ts, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def generate_cot_xml(lat, lon, hae_m=0.0, speed_ms=0.0, course_deg=0.0,
                     battery_pct=100.0, flight_mode="CRUISE",
                     callsign=DEFAULT_COT_CALLSIGN, uid=DEFAULT_COT_UID,
                     cot_type=DEFAULT_COT_TYPE, stale_s=20.0,
                     extra_remarks=""):
    """
    Generate a MIL-STD-compliant Cursor-on-Target (CoT) XML event string.
    """
    now = time.time()
    t_now_str   = iso_zulu(now)
    t_start_str = t_now_str
    t_stale_str = iso_zulu(now + stale_s)

    root = ET.Element("event", {
        "version": "2.0",
        "uid": str(uid),
        "type": str(cot_type),
        "how": "m-g",   # Machine generated, GPS / EKF derived
        "time": t_now_str,
        "start": t_start_str,
        "stale": t_stale_str,
    })

    safe_lat = float(lat if lat is not None else 0.0)
    safe_lon = float(lon if lon is not None else 0.0)
    safe_hae = float(hae_m if hae_m is not None else 0.0)
    safe_speed = float(speed_ms if speed_ms is not None else 0.0)
    safe_course = float(course_deg if course_deg is not None else 0.0)
    safe_batt = float(battery_pct if battery_pct is not None else 100.0)

    ET.SubElement(root, "point", {
        "lat": f"{safe_lat:.6f}",
        "lon": f"{safe_lon:.6f}",
        "hae": f"{safe_hae:.1f}",
        "ce": "5.0",   # Circular error estimate in metres
        "le": "3.0",   # Linear error estimate in metres
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "contact", {"callsign": str(callsign)})
    ET.SubElement(detail, "track", {
        "course": f"{safe_course % 360.0:.1f}",
        "speed": f"{safe_speed:.1f}",
    })
    ET.SubElement(detail, "status", {
        "battery": f"{int(max(0, min(100, safe_batt)))}",
    })

    remarks_text = f"MODE:{flight_mode} | BATT:{safe_batt:.0f}%"
    if extra_remarks:
        remarks_text += f" | {extra_remarks}"
    remarks = ET.SubElement(detail, "remarks")
    remarks.text = remarks_text

    return ET.tostring(root, encoding="utf-8", xml_declaration=True).decode("utf-8")


def parse_cot_xml(xml_content):
    """
    Parse a CoT XML message string.
    Returns a structured dictionary or None if invalid.
    """
    if isinstance(xml_content, bytes):
        try:
            xml_content = xml_content.decode("utf-8", errors="replace")
        except Exception:
            return None

    xml_content = xml_content.strip()
    if not xml_content.startswith("<"):
        return None

    try:
        root = ET.fromstring(xml_content)
        if root.tag != "event":
            return None

        uid      = root.attrib.get("uid", "")
        cot_type = root.attrib.get("type", "")
        how      = root.attrib.get("how", "")
        time_str = root.attrib.get("time", "")

        point = root.find("point")
        lat   = float(point.attrib.get("lat", 0.0)) if point is not None else 0.0
        lon   = float(point.attrib.get("lon", 0.0)) if point is not None else 0.0
        hae   = float(point.attrib.get("hae", 0.0)) if point is not None else 0.0

        detail = root.find("detail")
        callsign = ""
        speed    = 0.0
        course   = 0.0
        remarks  = ""

        if detail is not None:
            contact = detail.find("contact")
            if contact is not None:
                callsign = contact.attrib.get("callsign", "")

            track = detail.find("track")
            if track is not None:
                speed  = float(track.attrib.get("speed", 0.0))
                course = float(track.attrib.get("course", 0.0))

            rmk = detail.find("remarks")
            if rmk is not None and rmk.text:
                remarks = rmk.text.strip()

        # Identify ATAK intent (e.g. SPI, GeoChat, Tasking, Waypoint)
        is_spi = "b-m-p-s-p-i" in cot_type or "SPI" in remarks
        is_waypoint = "b-m-p-w" in cot_type or "WAYPOINT" in remarks or "ROUTE" in remarks

        return {
            "uid":         uid,
            "type":        cot_type,
            "how":         how,
            "time":        time_str,
            "lat":         lat,
            "lon":         lon,
            "hae_m":       hae,
            "callsign":    callsign,
            "speed_ms":    speed,
            "course_deg":  course,
            "remarks":     remarks,
            "is_spi":      is_spi,
            "is_waypoint": is_waypoint,
        }
    except Exception:
        return None


class MCOCoTBridge:
    """
    Background daemon for managing tactical CoT multicast and point-to-point links.
    """

    def __init__(self, multicast_group=DEFAULT_COT_MULTICAST_GROUP,
                 multicast_port=DEFAULT_COT_MULTICAST_PORT,
                 unicast_targets=None,
                 callsign=DEFAULT_COT_CALLSIGN,
                 uid=DEFAULT_COT_UID,
                 enabled=True):
        self.multicast_group = multicast_group
        self.multicast_port  = multicast_port
        self.unicast_targets = unicast_targets or []  # List of (ip, port)
        self.callsign        = callsign
        self.uid             = uid
        self.enabled         = enabled

        self._running        = False
        self._tx_sock        = None
        self._rx_sock        = None
        self._rx_thread      = None
        self._lock           = threading.Lock()

        # Telemetry state
        self._last_lat       = 0.0
        self._last_lon       = 0.0
        self._last_alt_m     = 0.0
        self._last_speed_ms  = 0.0
        self._last_course    = 0.0
        self._last_batt_pct  = 100.0
        self._last_mode      = "STANDBY"
        self._extra_remarks  = ""

        # Statistics & incoming queues
        self.n_packets_sent  = 0
        self.n_packets_recv  = 0
        self.received_waypoints = []  # List of (lat, lon, alt_m, callsign)
        self.latest_spi         = None
        self.last_rx_event      = None

    def start(self):
        """Initialize multicast TX and RX sockets."""
        if not self.enabled or self._running:
            return

        with self._lock:
            # Setup TX socket
            try:
                self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                # Multicast TTL: set to 1 for subnet or 16 for tactical mesh
                self._tx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 16)
                # Enable multicast loopback for local testing
                self._tx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
            except Exception as e:
                print(f"[CoT Bridge] TX socket init warning: {e}")

            # Setup RX socket
            try:
                self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except AttributeError:
                    pass

                self._rx_sock.bind(("", self.multicast_port))

                # Join multicast group
                mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
                self._rx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                self._rx_sock.settimeout(0.5)
            except Exception as e:
                print(f"[CoT Bridge] RX socket init warning: {e}")

            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            print(f"[CoT Bridge] Online: Multicast {self.multicast_group}:{self.multicast_port}")

    def _rx_loop(self):
        """Background receiver for inbound ATAK CoT packets."""
        while self._running:
            if not self._rx_sock:
                time.sleep(0.5)
                continue
            try:
                data, addr = self._rx_sock.recvfrom(65535)
                if not data:
                    continue

                parsed = parse_cot_xml(data)
                if not parsed:
                    continue

                # Ignore packets originating from this device's UID
                if parsed["uid"] == self.uid:
                    continue

                with self._lock:
                    self.n_packets_recv += 1
                    self.last_rx_event = parsed

                    # Check for Waypoint or SPI from ATAK operator
                    if parsed["is_waypoint"] or parsed["is_spi"]:
                        wp = (parsed["lat"], parsed["lon"], parsed["hae_m"], parsed["callsign"])
                        self.received_waypoints.append(wp)
                        if len(self.received_waypoints) > 50:
                            self.received_waypoints.pop(0)

                        if parsed["is_spi"]:
                            self.latest_spi = parsed

                        print(f"[CoT Bridge] Inbound ATAK Tasking: {parsed['callsign']} -> "
                              f"Lat:{parsed['lat']:.5f} Lon:{parsed['lon']:.5f} HAE:{parsed['hae_m']:.1f}m")

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    time.sleep(0.1)

    def update_telemetry(self, lat, lon, alt_m, speed_ms, course_deg,
                         battery_pct=100.0, flight_mode="CRUISE", extra_remarks=""):
        """Update current vehicle kinematic telemetry."""
        with self._lock:
            self._last_lat      = lat
            self._last_lon      = lon
            self._last_alt_m    = alt_m
            self._last_speed_ms = speed_ms
            self._last_course   = course_deg
            self._last_batt_pct = battery_pct
            self._last_mode     = flight_mode
            self._extra_remarks = extra_remarks

    def broadcast_telemetry(self):
        """Broadcast current vehicle state as a CoT XML message."""
        if not self._running or not self._tx_sock:
            return False

        with self._lock:
            lat   = self._last_lat
            lon   = self._last_lon
            alt   = self._last_alt_m
            speed = self._last_speed_ms
            crs   = self._last_course
            batt  = self._last_batt_pct
            mode  = self._last_mode
            rmk   = self._extra_remarks

        xml_str = generate_cot_xml(
            lat=lat, lon=lon, hae_m=alt, speed_ms=speed, course_deg=crs,
            battery_pct=batt, flight_mode=mode, callsign=self.callsign,
            uid=self.uid, extra_remarks=rmk
        )
        data = xml_str.encode("utf-8")

        success = False
        # Send Multicast
        try:
            self._tx_sock.sendto(data, (self.multicast_group, self.multicast_port))
            self.n_packets_sent += 1
            success = True
        except Exception:
            pass

        # Send Unicast endpoints if configured
        for ip, port in self.unicast_targets:
            try:
                self._tx_sock.sendto(data, (ip, port))
                success = True
            except Exception:
                pass

        return success

    def pop_latest_waypoint(self):
        """Retrieve and remove the oldest pending ATAK target waypoint."""
        with self._lock:
            if self.received_waypoints:
                return self.received_waypoints.pop(0)
            return None

    def get_status(self):
        """Return operational status dictionary."""
        with self._lock:
            return {
                "running":         self._running,
                "enabled":         self.enabled,
                "multicast_group": self.multicast_group,
                "multicast_port":  self.multicast_port,
                "callsign":        self.callsign,
                "uid":             self.uid,
                "packets_sent":    self.n_packets_sent,
                "packets_recv":    self.n_packets_recv,
                "pending_waypoints": len(self.received_waypoints),
                "latest_spi":      self.latest_spi,
                "last_lat":        self._last_lat,
                "last_lon":        self._last_lon,
                "last_alt_m":      self._last_alt_m,
                "flight_mode":     self._last_mode,
            }

    def close(self):
        """Shutdown sockets and background thread."""
        self._running = False
        with self._lock:
            if self._rx_sock:
                try:
                    self._rx_sock.close()
                except Exception:
                    pass
                self._rx_sock = None

            if self._tx_sock:
                try:
                    self._tx_sock.close()
                except Exception:
                    pass
                self._tx_sock = None


# Global singleton instance for easy import and access
_cot_bridge_instance = None

def get_cot_bridge(auto_start=True):
    global _cot_bridge_instance
    if _cot_bridge_instance is None:
        _cot_bridge_instance = MCOCoTBridge()
        if auto_start:
            _cot_bridge_instance.start()
    return _cot_bridge_instance
