#!/usr/bin/env bash
# =============================================================================
# probe_dds.sh -- lists the CycloneDDS applications and topics of the robot, from a laptop.
#
# Nothing to install on board: DDS discovery is peer-to-peer on the same subnet, so an ethernet
# connection is enough. It is PASSIVE: it only reads the builtin discovery topics, it publishes
# nothing. Requires the `cyclonedds` command line tool: pip install --user cyclonedds==0.10.2
#
# USAGE
#   ./probe_dds.sh                 # interface enp1s0, domain 0
#   ./probe_dds.sh enp1s0 0
#   ./probe_dds.sh enp1s0 0 peers  # when multicast does not get through (see below)
# =============================================================================
set -u
IFACE="${1:-enp1s0}"
DOMAIN="${2:-0}"
MODE="${3:-multicast}"
export PATH="$HOME/.local/bin:$PATH"

if [ "$MODE" = "peers" ]; then
  # fallback: unicast discovery towards the known onboard addresses, when multicast is blocked
  export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General>\
<Interfaces><NetworkInterface name=\"$IFACE\"/></Interfaces>\
<AllowMulticast>false</AllowMulticast></General>\
<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>\
<Peer address=\"192.168.123.161\"/><Peer address=\"192.168.123.162\"/>\
<Peer address=\"192.168.123.163\"/><Peer address=\"192.168.123.164\"/>\
</Peers></Discovery></Domain></CycloneDDS>"
else
  export CYCLONEDDS_URI="<CycloneDDS><Domain id=\"any\"><General>\
<Interfaces><NetworkInterface name=\"$IFACE\"/></Interfaces></General></Domain></CycloneDDS>"
fi

echo "interface $IFACE | domain $DOMAIN | discovery $MODE"
ip -br addr show "$IFACE" 2>/dev/null || echo "  !! interface $IFACE does not exist"
echo

echo "================ DDS APPLICATIONS ON THE NETWORK ================"
cyclonedds ps -i "$DOMAIN" -r 10s --suppress-progress-bar 2>&1

echo
echo "================ TOPICS (name + type + QoS) ================"
cyclonedds ls -i "$DOMAIN" -r 10s --suppress-progress-bar 2>&1

echo
echo "================ IDL OF THE KEY TOPICS ================"
# Checks that the TYPE matches on both sides: an IDL mismatch produces no error, the data just
# never arrives.
for t in rt/lowstate rt/lowcmd rt/wirelesscontroller; do
  echo "--- $t ---"
  cyclonedds typeof "$t" -i "$DOMAIN" -r 5s --suppress-progress-bar 2>&1 | head -40
done
