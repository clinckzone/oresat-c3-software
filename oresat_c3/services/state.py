"""
C3 State Service

This handles the main C3 state machine and saving state.
"""

import os
import subprocess
from time import monotonic, time

import canopen
from olaf import NodeStop, Service, UpdaterState, logger

from .. import C3State
from ..drivers.fm24cl64b import Fm24cl64b
from ..subsystems.antennas import Antennas
from ..subsystems.rtc import set_rtc_time


class StateService(Service):
    """
    Manages the satellite's operational state machine, including state transitions,
    health monitoring, and persisting state to F-RAM.
    """

    # Safe battery pack voltage (in mV).
    BAT_LEVEL_LOW = 6500

    # I2C bus number for the F-RAM.
    I2C_BUS_NUM = 2

    # F-RAM I2C address.
    FRAM_I2C_ADDR = 0x50

    def __init__(self, fram_objs: list, mock_hw: bool = False):
        super().__init__()

        self._fram_objs = fram_objs
        self._fram = Fm24cl64b(self.I2C_BUS_NUM, self.FRAM_I2C_ADDR, mock_hw)
        self._antennas = Antennas(mock_hw)

        # --- Service State ---
        self._attempts = 0  # Total deployment attempts so far.
        self._loops = 0  # Loop counter for periodic tasks.
        self._last_state = C3State.OFFLINE  # Previous state of the satellite.
        self._last_antennas_deploy = 0  # Timestamp of the last antenna deployment attempt.
        self._start_time = monotonic()  # Service start time.

        # --- CANopen Object Dictionary References ---
        # These are initialized in on_start() with live references to the node's OD.
        self._c3_state_obj: canopen.objectdictionary.Variable = None  # Current C3 state.
        self._reset_timeout_obj: canopen.objectdictionary.Variable = None  # Auto-reset timeout in flight mode.
        self._attempts_obj: canopen.objectdictionary.Variable = None  # Max antenna deployment attempts.
        self._deployed_obj: canopen.objectdictionary.Variable = None  # Antenna deployment status flag.
        self._pre_deploy_timeout_obj: canopen.objectdictionary.Variable = None  # PRE_DEPLOY state timeout.
        self._ant_attempt_timeout_obj: canopen.objectdictionary.Variable = None  # Antenna burn attempt duration.
        self._ant_attempt_between_timeout_obj: canopen.objectdictionary.Variable = None  # Wait time between burn attempts.
        self._ant_reattempt_timeout_obj: canopen.objectdictionary.Variable = None  # Wait time between burn sequences.
        self._tx_timeout_obj: canopen.objectdictionary.Variable = None  # Transmission timeout.
        self._tx_enable_obj: canopen.objectdictionary.Variable = None  # Transmission enable flag.
        self._last_tx_enable_obj: canopen.objectdictionary.Variable = None  # Timestamp of last transmission enable.
        self._last_edl_obj: canopen.objectdictionary.Variable = None  # Timestamp of last EDL packet reception.
        self._edl_timeout_obj: canopen.objectdictionary.Variable = None  # EDL mode timeout.
        self._vbatt_bp1_obj: canopen.objectdictionary.Variable = None  # Battery pack 1 voltage.
        self._vbatt_bp2_obj: canopen.objectdictionary.Variable = None  # Battery pack 2 voltage.

    def on_start(self):
        """
        Initializes the service, restores state from F-RAM, and sets up
        the initial satellite state.
        """
        edl_rec = self.node.od["edl"]
        antennas_rec = self.node.od["antennas"]
        tx_control_rec = self.node.od["tx_control"]
        bat_1_rec = self.node.od["battery_1"]

        # Initialize live references to the C3 Node's Object Dictionary.
        self._c3_state_obj = self.node.od["status"]
        self._reset_timeout_obj = self.node.od["reset_timeout"]
        self._attempts_obj = antennas_rec["attempts"]
        self._deployed_obj = antennas_rec["deployed"]
        self._pre_deploy_timeout_obj = antennas_rec["pre_attempt_timeout"]
        self._ant_attempt_timeout_obj = antennas_rec["attempt_timeout"]
        self._ant_attempt_between_timeout_obj = antennas_rec["attempt_between_timeout"]
        self._ant_reattempt_timeout_obj = antennas_rec["reattempt_timeout"]
        self._tx_timeout_obj = tx_control_rec["timeout"]
        self._tx_enable_obj = tx_control_rec["enable"]
        self._last_tx_enable_obj = tx_control_rec["last_enable_timestamp"]
        self._last_edl_obj = edl_rec["last_timestamp"]
        self._edl_timeout_obj = edl_rec["timeout"]
        self._vbatt_bp1_obj = bat_1_rec["pack_1_vbatt"]
        self._vbatt_bp2_obj = bat_1_rec["pack_2_vbatt"]

        # Restore the previous state from F-RAM.
        self.restore_state()

        # --- Sanitize Restored State ---
        # The following checks ensure the satellite starts in a safe and consistent state.

        # If tx_enable is false, the last_tx_enable_timestamp should be 0.
        if not self._tx_enable_obj.value:
            self._last_tx_enable_obj.value = 0

        # The EDL (Emergency Downlink) state should not persist across restarts.
        # If the satellite was in EDL, transition it to STANDBY.
        if self._c3_state_obj.value == C3State.EDL:
            self._c3_state_obj.value = C3State.STANDBY.value

        # Register a callback that updates the last_tx_enable_timestamp when tx_enable is changed.
        self.node.add_sdo_callbacks("tx_control", "enable", None, self._on_write_tx_enable)

        # On a fresh F-RAM, the state will be invalid.
        # In this case, set the initial state to PRE_DEPLOY.
        if self._c3_state_obj.value not in list(C3State):
            self._c3_state_obj.value = C3State.PRE_DEPLOY.value

        self._last_state = self._c3_state_obj.value
        logger.info(f"C3 initial state: {C3State(self._last_state).name}")

        # Set the service start time
        self._start_time = monotonic()

    def on_stop(self):
        """Saves the current state to F-RAM before the service stops."""
        self.store_state()

    def _on_write_tx_enable(self, data: bool):
        """
        Callback for when the 'tx_control.enable' SDO is written to.
        Updates the tx_enable flag and the last_enable_timestamp.
        """
        self._tx_enable_obj.value = data
        if data:
            logger.info("enabling tx")
            self._last_tx_enable_obj.value = int(time())
        else:
            logger.info("disabling tx")
            self._last_tx_enable_obj.value = 0

    def _reset(self):
        """
        Performs a system reset, prioritizing a graceful shutdown before
        resorting to a hard reset.
        """
        # Do not reset if a software update is in progress.
        if self.node.od["updater"]["status"].value == UpdaterState.UPDATING:
            return

        logger.info("system reset")

        # Attempt a graceful reset by stopping the watchdog service.
        # This allows other services to shut down cleanly.
        result = subprocess.run(
            ["systemctl", "stop", "oresat-c3-watchdog"],
            shell=True,
            check=False,
            capture_output=True,
        )

        # If the graceful shutdown fails, perform a hard reset.
        if result.returncode != 0:
            logger.error("stopping watchdog app failed, doing a hard reset")
            # NodeStop.HARD_RESET triggers an immediate, non-graceful system reboot,
            # similar to a hardware reset. The on_stop() method will likely not be
            # called, and state may not be saved.
            self.node.stop(NodeStop.HARD_RESET)

    def _pre_deploy(self):
        """
        Handles the PRE_DEPLOY state. In this state, the satellite waits for a
        configured duration before attempting to deploy the antennas.
        During this time, it enables beacon transmissions.
        """
        # Wait for the pre-deploy timeout before moving to the DEPLOY state.
        if (monotonic() - self._start_time) < self._pre_deploy_timeout_obj.value:
            # Enable beacon transmissions if they are not already active.
            if not self._tx_enable_obj.value:
                self._tx_enable_obj.value = True
                self._last_tx_enable_obj.value = int(time())
        else:
            # Timeout reached, transition to DEPLOY state.
            logger.info("pre-deploy timeout reached")
            self._c3_state_obj.value = C3State.DEPLOY.value

    def _deploy(self):
        """
        Handles the DEPLOY state. This method attempts to deploy the antennas
        until successful or until the maximum number of attempts is reached.
        """
        # Continue attempting deployment as long as the antennas are not deployed
        # and the attempt limit has not been reached.
        if not self._deployed_obj.value and self._attempts < self._attempts_obj.value:
            # Check if enough time has passed since the last attempt and if battery levels are sufficient.
            if (
                monotonic() > (self._last_antennas_deploy + self._ant_reattempt_timeout_obj.value)
                and self.is_bat_lvl_good
            ):
                # Attempt to deploy the antennas.
                logger.info(f"deploying antennas, attempt {self._attempts + 1}")
                self._antennas.deploy(
                    self._ant_attempt_timeout_obj.value,
                    self._ant_attempt_between_timeout_obj.value,
                )
                self._last_antennas_deploy = monotonic()
                self._attempts += 1
        else:
            # Stop deployment attempts if antennas are deployed or max attempts are reached.
            # Transition to STANDBY state.
            logger.info("antennas deployed")
            self._c3_state_obj.value = C3State.STANDBY.value
            self._deployed_obj.value = True
            self._attempts = 0

    def _standby(self):
        """
        STANDBY is the default operational state where the satellite is idle and
        monitoring its health and the environment. It checks for critical events
        in a specific priority order.
        """
        # 1. Check for EDL timeout: If an Emergency Downlink is active,
        #    immediately switch to EDL state.
        if self.has_edl_timed_out:
            self._c3_state_obj.value = C3State.EDL.value
        # 2. Check for reset timeout: If the system has been running for too long
        #    without a reset (in flight mode), trigger a system reset for safety.
        elif self.has_reset_timed_out:
            self._reset()
        # 3. Check for beaconing conditions: If transmission hasn't timed out and
        #    the battery is good, switch to BEACON state to transmit data.
        elif not self.has_tx_timed_out and self.is_bat_lvl_good:
            self._c3_state_obj.value = C3State.BEACON.value
        # Otherwise, remain in STANDBY.

    def _beacon(self):
        """
        BEACON is the active transmission state. The satellite sends out beacon
        signals with telemetry data. It continuously checks conditions to ensure
        it's safe to keep transmitting.
        """
        # 1. Check for EDL timeout: If an Emergency Downlink is triggered,
        #    immediately switch to EDL state, overriding beaconing.
        if self.has_edl_timed_out:
            self._c3_state_obj.value = C3State.EDL.value
        # 2. Check for reset timeout: If the system has been running for too long,
        #    trigger a reset, even during beaconing.
        elif self.has_reset_timed_out:
            self._reset()
        # 3. Check for exit conditions: If the transmitter has been on for too long
        #    or if the battery level drops, stop beaconing and return to STANDBY.
        elif self.has_tx_timed_out or not self.is_bat_lvl_good:
            self._c3_state_obj.value = C3State.STANDBY.value
        # Otherwise, continue beaconing.

    def _edl(self):
        """
        EDL (Emergency Downlink) is a high-priority state triggered to ensure
        communication with the ground. The satellite will stay in this mode
        until the emergency condition is cleared.
        """
        # If the EDL timeout has expired, transition to a new state.
        if not self.has_edl_timed_out:
            # If power is good and transmission hasn't timed out, go to BEACON.
            if not self.has_tx_timed_out and self.is_bat_lvl_good:
                self._c3_state_obj.value = C3State.BEACON.value
            # Otherwise, fall back to the safe STANDBY state.
            else:
                self._c3_state_obj.value = C3State.STANDBY.value

    def on_loop(self):
        """
        The main loop of the state machine, executed repeatedly.
        It manages state transitions, checks for timeouts, and saves the state periodically.
        """
        # Disable transmission if it has been active for too long. This is a safety
        # measure to prevent continuous transmission due to power constraints or
        # regulatory requirements.
        if self.has_tx_timed_out and self._tx_enable_obj.value:
            logger.info("tx enable timeout")
            self._tx_enable_obj.value = False

        # Log any state changes that occur (e.g., through the REST API).
        state_a = self._c3_state_obj.value
        if state_a != self._last_state:
            logger.info(f"tx en: {self._tx_enable_obj.value} | bat good: {self.is_bat_lvl_good}")
            logger.info(
                f"C3 state change: {C3State(self._last_state).name} -> {C3State(state_a).name}"
            )

        # Execute the handler for the current state.
        if self._c3_state_obj.value == C3State.PRE_DEPLOY:
            self._pre_deploy()
        elif self._c3_state_obj.value == C3State.DEPLOY:
            self._deploy()
        elif self._c3_state_obj.value == C3State.STANDBY:
            self._standby()
        elif self._c3_state_obj.value == C3State.BEACON:
            self._beacon()
        elif self._c3_state_obj.value == C3State.EDL:
            self._edl()
        else:
            # Safety fallback: If the state is invalid, reset to PRE_DEPLOY.
            logger.error(f"C3 invalid state: {self._c3_state_obj.value}, resetting to PRE_DEPLOY")
            self._c3_state_obj.value = C3State.PRE_DEPLOY.value
            self._last_state = self._c3_state_obj.value
            return

        # Log any state changes that occurred within the state handler.
        self._last_state = self._c3_state_obj.value
        if state_a != self._last_state:
            logger.info(f"tx en: {self._tx_enable_obj.value} | bat good: {self.is_bat_lvl_good}")
            logger.info(
                f"C3 state change: {C3State(state_a).name} -> {C3State(self._last_state).name}"
            )

        # Save the current state to F-RAM periodically.
        self._loops += 1
        self._loops %= 10  # This loop runs every 0.1s, so this triggers every 1s.
        if self._loops == 0:
            self.store_state()

        # The loop is called every 0.1 seconds.
        self.sleep(0.1)

    @property
    def has_tx_timed_out(self) -> bool:
        """
        Checks if the transmission has been enabled for longer than the allowed timeout.
        """
        # Returns True if the time since the last transmission enable exceeds the timeout.
        return (time() - self._last_tx_enable_obj.value) > self._tx_timeout_obj.value

    @property
    def has_edl_timed_out(self) -> bool:
        """
        Checks if the EDL (Emergency Downlink) mode is currently active.

        Note: The name is slightly misleading. This property returns True if the
        time since the last EDL packet was received is *less than* the EDL timeout,
        meaning EDL mode is *active*. A better name might be `is_edl_active`.
        """
        # Returns True if the time since the last EDL packet is within the timeout period.
        return (time() - self._last_edl_obj.value) < self._edl_timeout_obj.value

    @property
    def is_bat_lvl_good(self) -> bool:
        """Checks if the battery levels are sufficient for normal operations."""
        # Returns True only if both battery packs are above the minimum voltage level.
        return (
            self._vbatt_bp1_obj.value > self.BAT_LEVEL_LOW
            and self._vbatt_bp2_obj.value > self.BAT_LEVEL_LOW
        )

    @property
    def has_reset_timed_out(self) -> bool:
        """
        Checks if the system needs to be reset due to prolonged uptime in flight mode.
        """
        # Reset is only triggered if running as root in flight mode (not in testing).
        if os.geteuid() != 0 or not self.node.od["flight_mode"].value:
            return False

        # Returns True if the uptime exceeds the configured reset timeout.
        return (monotonic() - self._start_time) > self._reset_timeout_obj.value

    def store_state(self):
        """
        Stores the values of all registered F-RAM objects to the F-RAM chip.
        """
        # Do not store state in PRE_DEPLOY, as it's a transient startup state.
        if self._c3_state_obj.value == C3State.PRE_DEPLOY:
            return

        offset = 0
        for obj in self._fram_objs:
            if obj.data_type == canopen.objectdictionary.DOMAIN:
                continue

            if obj.data_type == canopen.objectdictionary.OCTET_STRING:
                raw = obj.value
                raw_len = len(obj.default)
            else:
                raw = obj.encode_raw(obj.value)
                raw_len = len(raw)

            self._fram.write(offset, raw)
            offset += raw_len

    def restore_state(self):
        """
        Restores the state from F-RAM by reading the values of all
        registered F-RAM objects.
        """
        offset = 0
        for obj in self._fram_objs:
            if obj.data_type == canopen.objectdictionary.DOMAIN:
                continue

            if obj.data_type == canopen.objectdictionary.OCTET_STRING:
                size = len(obj.default)
                obj.value = self._fram.read(offset, size)
            else:
                size = len(obj.encode_raw(obj.default))
                raw = self._fram.read(offset, size)
                # If F-RAM is empty, the status will be empty. Default to PRE_DEPLOY.
                if obj.name == "status":
                    raw = raw or C3State.PRE_DEPLOY.value.to_bytes(1, "little")
                obj.value = obj.decode_raw(raw)
            offset += size

    def clear_state(self):
        """
        Clears the F-RAM, except for cryptographic keys, and resets the RTC time.
        """
        self._fram.clear()

        offset = 0
        for obj in self._fram_objs:
            if obj.data_type == canopen.objectdictionary.DOMAIN:
                continue

            # Preserve cryptographic keys during a state clear.
            if obj.name.startswith("crypto_key"):
                raw = obj.value
                raw_len = len(obj.default)
                self._fram.write(offset, raw)
            else:
                # For other objects, the length is determined by their raw encoding.
                raw = obj.encode_raw(obj.value)
                raw_len = len(raw)

            offset += raw_len

        set_rtc_time(0)
