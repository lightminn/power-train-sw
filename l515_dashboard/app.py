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

    def __init__(self, client, *, poll_interval_s=1.0):
        super().__init__(); self.client=client; self.poll_interval_s=poll_interval_s
        self.streaming_enabled=True; self._confirming=False

    def compose(self)->ComposeResult:
        yield Header(); yield Vertical(Static("DISCONNECTED",id="status"), id="body"); yield Footer()

    def on_mount(self): self.set_interval(self.poll_interval_s,self.refresh_status)
    def refresh_status(self):
        snap=self.client.poll()
        if snap: self.show_status(snap.payload)
        elif self.client.last_error: self.query_one("#status",Static).update(f"DISCONNECTED\n{self.client.last_error}")

    def show_status(self,p):
        sdk=p.get("sdk",{}); srt=p.get("srt",{}); system=p.get("system",{}); ros=p.get("ros_publish_counts",{})
        self.streaming_enabled=bool(srt.get("enabled",False))
        text=(f"State: {p.get('state','?')}\nSDK: serial={sdk.get('serial')} profile={sdk.get('profile')} source={sdk.get('source_state')}\n"
              f"ROS: {dict(ros)}\nSRT: running={srt.get('running')} enabled={srt.get('enabled')} mode={srt.get('mode')} sent={srt.get('sent')} dropped={srt.get('dropped')} client={srt.get('client_state')}\n"
              f"Resources: CPU={system.get('cpu_percent')}% RSS={system.get('current_rss_bytes')}\nErrors: {p.get('last_error') or srt.get('last_error') or '-'}")
        self.query_one("#status",Static).update(text)

    def _command(self,kind,payload=None):
        try:
            snap=self.client.request(kind,payload or {})
            if snap and kind != "stop_gateway": self.show_status(snap.payload)
        except Exception as exc: self.query_one("#status",Static).update(f"Command failed: {exc}")
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
            if yes: self._command("stop_gateway"); self.exit()

