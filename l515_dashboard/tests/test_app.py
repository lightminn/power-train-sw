import asyncio

from l515_dashboard.app import DashboardApp


class Client:
    def __init__(self, result=None, error=None, poll_result=None):
        self.commands=[]; self.result=result; self.error=error; self.last_error=None
        self.poll_result=poll_result; self.poll_count=0
    def poll(self):
        self.poll_count += 1
        if self.error: raise self.error
        return self.poll_result
    def request(self, kind, payload=None):
        self.commands.append((kind, payload or {}))
        if self.error: raise self.error
        return self.result


def test_dashboard_renders_status_and_keys():
  async def scenario():
    client=Client(); app=DashboardApp(client, poll_interval_s=60)
    async with app.run_test() as pilot:
        app.show_status({"state":"DEGRADED", "sdk":{"serial":"f0271544", "profile":"1280x720", "native_callback_rates_hz":{"color":29.9}},
          "ros_topic_rates_hz":{"/l515/color/image_raw":29.8},
          "ros_publish_counts":{"/l515/color/image_raw":12}, "srt":{"running":True,"enabled":True,"mode":"rgb","sent":10,"dropped":2},
          "system":{"cpu_percent":3.5,"current_rss_bytes":1048576}, "last_error":"camera lost"})
        await pilot.pause()
        text=app.query_one("#status").render().plain
        for value in ("DEGRADED","f0271544","1280x720","Native Hz","29.9","ROS Hz","29.8","12","rgb","10","2","3.5","camera lost"): assert value in text
        for key in ("1","2","3","s","r"): await pilot.press(key)
        assert client.commands == [("set_video_mode",{"mode":"rgb"}), ("set_video_mode",{"mode":"depth"}),
          ("set_video_mode",{"mode":"overlay"}), ("set_streaming",{"enabled":False}), ("restart_gateway",{})]
  asyncio.run(scenario())


def test_q_exits_client_and_shift_q_requires_confirmation():
  async def scenario():
    client=Client(); app=DashboardApp(client, poll_interval_s=60)
    async with app.run_test() as pilot:
        await pilot.press("q"); await pilot.pause()
    assert not client.commands
    client=Client(); app=DashboardApp(client, poll_interval_s=60)
    async with app.run_test() as pilot:
        await pilot.press("Q"); await pilot.pause()
        assert app.query_one("#confirm-stop")
        await pilot.press("y"); await pilot.pause()
        assert app.is_running
    assert client.commands == [("stop_gateway",{})]

    class Ack: acknowledged=True; payload={"accepted":True}
    client=Client(Ack()); app=DashboardApp(client,poll_interval_s=60)
    async with app.run_test() as pilot:
        await pilot.press("Q","y"); await pilot.pause()
    assert client.commands == [("stop_gateway",{})]
  asyncio.run(scenario())


def test_failed_stop_stays_open_and_displays_error():
  async def scenario():
    client=Client(error=TimeoutError("ack timeout")); app=DashboardApp(client,poll_interval_s=60)
    async with app.run_test() as pilot:
        await pilot.press("Q","y"); await pilot.pause()
        assert app.is_running
        assert "ack timeout" in app.query_one("#status").render().plain
  asyncio.run(scenario())


def test_gateway_and_observability_are_polled_independently():
  async def scenario():
    class Snapshot:
      def __init__(self, payload): self.payload=payload

    gateway=Client(poll_result=Snapshot({"state":"RUNNING","sdk":{},"srt":{},"system":{}}))
    observability=Client(error=ConnectionError("observability offline"))
    app=DashboardApp(gateway,observability_client=observability,poll_interval_s=60)
    async with app.run_test() as pilot:
      app.refresh_status(); await pilot.pause()
      assert "RUNNING" in app.query_one("#status").render().plain
      assert "observability offline" in app.query_one("#observability-status").render().plain
      assert gateway.poll_count == 1 and observability.poll_count == 1

    gateway=Client(error=ConnectionError("gateway offline"))
    observability=Client(poll_result=Snapshot({
      "run_id":"run-1", "drop_count":2, "health":{"status":"DEGRADED"},
      "recent_events":{
        "COMMAND_OWNER":{"payload":{"owner":"autonomy"}},
        "ESTOP":{"payload":{"source":"us100"}},
        "FSM_TRANSITION":{"payload":{"segment":"DOOR","to":"WAIT_ARM"}},
        "MISSION":{"payload":{"result":"ARRIVED_DOOR"}},
        "ARM_RESULT":{"payload":{"result":"DONE"}},
      },
      "channel_health":{"l515_srt":{"status":"OK"},"arm_srt":{"status":"DEGRADED"}},
    }))
    app=DashboardApp(gateway,observability_client=observability,poll_interval_s=60)
    async with app.run_test() as pilot:
      app.refresh_status(); await pilot.pause()
      text=app.query_one("#observability-status").render().plain
      for value in ("run-1","DEGRADED","2","autonomy","us100","DOOR","WAIT_ARM",
                    "ARRIVED_DOOR","DONE","l515_srt","arm_srt"):
        assert value in text
      assert gateway.poll_count == 1 and observability.poll_count == 1
  asyncio.run(scenario())


def test_quit_and_confirmed_gateway_stop_never_command_observability():
  async def scenario():
    observability=Client()
    gateway=Client(); app=DashboardApp(gateway,observability_client=observability,poll_interval_s=60)
    async with app.run_test() as pilot:
      await pilot.press("q"); await pilot.pause()
    assert observability.commands == []

    class Ack: acknowledged=True; payload={"accepted":True}
    observability=Client(); gateway=Client(Ack())
    app=DashboardApp(gateway,observability_client=observability,poll_interval_s=60)
    async with app.run_test() as pilot:
      await pilot.press("Q","y"); await pilot.pause()
    assert gateway.commands == [("stop_gateway",{})]
    assert observability.commands == []
  asyncio.run(scenario())
