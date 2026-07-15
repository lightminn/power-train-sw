"""Textual presentation layer; intentionally contains no SDK or ROS imports."""

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.widgets import Footer, Header, Static


class DashboardApp(App):
    TITLE="L515 Gateway Dashboard"
    BINDINGS=[Binding("q","quit_client","Quit"), Binding("Q","confirm_gateway_stop","Stop Gateway"),
              Binding("1","mode_rgb","RGB"),Binding("2","mode_depth","Depth"),Binding("3","mode_overlay","Overlay"),
              Binding("s","toggle_streaming","Stream"),Binding("r","restart","Restart")]
    CSS="#confirm-stop { color: red; border: heavy red; padding: 1; }"

    def __init__(self, client, *, observability_client=None, poll_interval_s=1.0):
        super().__init__(); self.client=client; self.observability_client=observability_client
        self.poll_interval_s=poll_interval_s
        self.streaming_enabled=True; self._confirming=False

    def compose(self)->ComposeResult:
        yield Header()
        yield Vertical(
            Static("DISCONNECTED",id="status"),
            Static("OBSERVABILITY DISCONNECTED",id="observability-status"),
            id="body",
        )
        yield Footer()

    def on_mount(self): self.set_interval(self.poll_interval_s,self.refresh_status)
    def refresh_status(self):
        try:
            snap=self.client.poll()
            if snap: self.show_status(snap.payload)
            elif self.client.last_error:
                self.query_one("#status",Static).update(f"DISCONNECTED\n{self.client.last_error}")
        except Exception as exc:
            self.query_one("#status",Static).update(f"DISCONNECTED\n{exc}")

        if self.observability_client is None:
            return
        try:
            snap=self.observability_client.poll()
            if snap: self.show_observability_status(snap.payload)
            elif self.observability_client.last_error:
                self.query_one("#observability-status",Static).update(
                    f"OBSERVABILITY DISCONNECTED\n{self.observability_client.last_error}"
                )
        except Exception as exc:
            self.query_one("#observability-status",Static).update(
                f"OBSERVABILITY DISCONNECTED\n{exc}"
            )

    def show_status(self,p):
        sdk=p.get("sdk",{}); srt=p.get("srt",{}); system=p.get("system",{}); ros=p.get("ros_publish_counts",{})
        native_rates=sdk.get("native_callback_rates_hz",{}); ros_rates=p.get("ros_topic_rates_hz",{})
        self.streaming_enabled=bool(srt.get("enabled",False))
        text=(f"State: {p.get('state','?')}\nSDK: serial={sdk.get('serial')} profile={sdk.get('profile')} source={sdk.get('source_state')}\n"
              f"Native Hz: {dict(native_rates)}\nROS: {dict(ros)}\nROS Hz: {dict(ros_rates)}\nSRT: running={srt.get('running')} enabled={srt.get('enabled')} mode={srt.get('mode')} sent={srt.get('sent')} dropped={srt.get('dropped')} submit/sent/drop Hz={srt.get('submitted_rate_hz')}/{srt.get('sent_rate_hz')}/{srt.get('drop_rate_hz')} aligned-depth age={srt.get('aligned_depth_age_ms')} ms client={srt.get('client_state')}\n"
              f"Resources: CPU={system.get('cpu_percent')}% RSS={system.get('current_rss_bytes')}\nErrors: {p.get('last_error') or srt.get('last_error') or '-'}")
        self.query_one("#status",Static).update(text)

    @staticmethod
    def _event_payload(recent_events, event_type):
        event=recent_events.get(event_type,{})
        payload=event.get("payload",{}) if hasattr(event,"get") else {}
        return payload if hasattr(payload,"get") else {}

    def show_observability_status(self,p):
        health=p.get("health",{}); recent=p.get("recent_events",{})
        owner=self._event_payload(recent,"COMMAND_OWNER")
        estop=self._event_payload(recent,"ESTOP")
        hold=self._event_payload(recent,"MOTION_HOLD")
        stop_source=(estop or hold)
        fsm=self._event_payload(recent,"FSM_TRANSITION")
        mission=self._event_payload(recent,"MISSION")
        arm=self._event_payload(recent,"ARM_RESULT")
        channels=p.get("channel_health",{})
        can_health=self._event_payload(recent,"CAN_HEALTH")
        can_lines=self._format_can_health(can_health)
        text=(
            f"Observability: run={p.get('run_id','?')} health={health.get('status','?')} "
            f"drops={p.get('drop_count',0)}\n"
            f"Command owner: {owner.get('owner') or owner.get('command_owner') or '-'}\n"
            f"Hold/E-stop source: {stop_source.get('source') or stop_source.get('reason') or '-'}\n"
            f"Segment/FSM: {fsm.get('segment') or '-'} / "
            f"{fsm.get('to') or fsm.get('state') or '-'}\n"
            f"Mission result: {mission.get('result') or mission.get('state') or '-'}\n"
            f"Arm result: {arm.get('result') or arm.get('state') or '-'}\n"
            f"Channel health: {dict(channels)}\n"
            f"{can_lines}"
        )
        self.query_one("#observability-status",Static).update(text)

    @staticmethod
    def _format_can_health(payload):
        if not payload:
            return "CAN matrix: -"
        owner=payload.get("owner") or {}
        bus=payload.get("bus") or {}
        interlock=payload.get("interlock") or {}
        consistency=payload.get("wheel_consistency") or {}
        lines=[
            "CAN owner: pid=%s process=%s lock=%s acquired=%s" % (
                owner.get("pid","-"),owner.get("process_name","-"),
                owner.get("lock_path","-"),owner.get("acquisition_time","-"),
            ),
            "CAN bus: rxΔ=%s txΔ=%s warning=%s passive=%s bus-offΔ=%s restarts=%s" % (
                bus.get("rx_packet_delta",0),bus.get("tx_packet_delta",0),
                bus.get("error_warning",False),bus.get("error_passive",False),
                bus.get("bus_off_delta",0),bus.get("restart_count",0),
            ),
            "Interlock: holds=%s estops=%s reset-required=%s" % (
                list(interlock.get("motion_hold_sources",())),
                list(interlock.get("latched_estop_sources",())),
                interlock.get("reset_required",False),
            ),
            "AK matrix:",
        ]
        for node in payload.get("ak_nodes",()):
            lines.append(
                "AK%s %s age=%s ms rate=%s Hz fault=%s stale=%s recovery=%s" % (
                    node.get("can_id","?"),node.get("physical_wheel","?"),
                    node.get("last_feedback_age_ms"),node.get("feedback_rate_hz"),
                    node.get("steer_fault",0),node.get("stale",False),
                    node.get("recovery_count",0),
                )
            )
        lines.append("ODrive matrix:")
        for node in payload.get("odrive_nodes",()):
            lines.append(
                "OD%s %s heartbeat-age=%s ms encoder-age=%s ms state=%s error=%s stale=%s recovery=%s" % (
                    node.get("node_id","?"),node.get("physical_wheel","?"),
                    node.get("last_heartbeat_age_ms"),node.get("last_encoder_age_ms"),
                    node.get("axis_state",0),node.get("axis_error",0),
                    node.get("stale",False),node.get("recovery_count",0),
                )
            )
        warning_labels=[
            "%s:%s" % (warning.get("severity","WARN"),warning.get("code","?"))
            for warning in consistency.get("warnings",())
        ]
        lines.append(
            "Wheel consistency: %s speed cap=%s" % (
                warning_labels or ["OK"],consistency.get("terrain_speed_cap",1.0),
            )
        )
        return "\n".join(lines)

    def _command(self,kind,payload=None):
        try:
            snap=self.client.request(kind,payload or {})
            if snap and kind != "stop_gateway": self.show_status(snap.payload)
            return snap
        except Exception as exc:
            self.query_one("#status",Static).update(f"Command failed: {exc}")
            return None
    def action_mode_rgb(self): self._command("set_video_mode",{"mode":"rgb"})
    def action_mode_depth(self): self._command("set_video_mode",{"mode":"depth"})
    def action_mode_overlay(self): self._command("set_video_mode",{"mode":"overlay"})
    def action_toggle_streaming(self): self._command("set_streaming",{"enabled":not self.streaming_enabled})
    def action_restart(self): self._command("restart_gateway")
    def action_quit_client(self): self.exit()
    def action_confirm_gateway_stop(self):
        if self._confirming: return
        self._confirming=True; self.query_one("#body").mount(Static("Stop Gateway and reap SRT? y/N",id="confirm-stop"))
    def on_key(self,event):
        if self._confirming and event.key.lower() in ("y","n","escape"):
            event.stop(); yes=event.key.lower()=="y"; self._confirming=False
            self.query_one("#confirm-stop").remove()
            if yes:
                snapshot = self._command("stop_gateway")
                if snapshot is not None and snapshot.acknowledged:
                    self.exit()
                elif snapshot is not None:
                    self.query_one("#status", Static).update(
                        "Gateway stop was not acknowledged; Dashboard remains connected"
                    )
