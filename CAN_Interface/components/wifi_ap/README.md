# wifi_ap - DEPRECATED

## Status

**This component has been removed from the CAN Interface Node.**

The CAN Interface Node is a headless backend that communicates exclusively
with the Display Node over a wired UART link (USB-C cable). All user-facing
configuration, log download, and system status functionality has been moved
to the **Display Node's `wifi_manager` component**.

## How Configuration Works Now

1. User connects to the Display Node's WiFi AP
2. Display Node web UI shows both local settings and CAN node settings
3. CAN node settings are proxied over the UART link using `CONFIG_CMD` /
   `CONFIG_RESP` messages defined in `shared/comm_protocol`

## Why Removed

- Eliminates WiFi radio usage on the CAN node (saves power, reduces RF noise)
- Simplifies the CAN node to focus solely on CAN bus communication
- Single WiFi AP for the user to connect to (less confusing)
- The Display Node has a touch screen for local interaction, making a separate
  WiFi AP on the CAN node redundant
