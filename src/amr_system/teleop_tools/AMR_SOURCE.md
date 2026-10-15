# Vendored teleop_tools packages

Copied from the local F1TENTH teleop_tools source without runtime changes.
Upstream: https://github.com/ros-teleop/teleop_tools
package.xml version: 1.2.1. Original copyright and license headers are retained.
Only joy_teleop and its teleop_tools_msgs dependency are included.
AMR axis mapping is in amr_stack; deadman release and timeouts are in amr_control.

Source: /home/acus02/f1tenth_ws/src/f1tenth_system/teleop_tools
SHA256 of the original copied runtime source (preserved in the transfer archive):

29489a6f909bad1bd207070e5d2b1d1359ca00a570f7b0a10a311d419eb161cd  joy_teleop/joy_teleop/joy_teleop.py

The Jazzy port keeps the local neutral-button `default` behavior. Runtime changes
are limited to comment formatting; the inherited upstream test now verifies this
local behavior instead of expecting the unmodified upstream startup exception.
