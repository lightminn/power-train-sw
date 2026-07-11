import asyncio

from l515_dashboard.app import DashboardApp


class Client:
    def __init__(self): self.commands=[]
    def poll(self): return None
    def request(self, kind, payload=None): self.commands.append((kind, payload or {}))


def test_dashboard_renders_status_and_keys():
  async def scenario():
    client=Client(); app=DashboardApp(client, poll_interval_s=60)
    async with app.run_test() as pilot:
        app.show_status({"state":"DEGRADED", "sdk":{"serial":"f0271544", "profile":"1280x720"},
          "ros_publish_counts":{"/l515/color/image_raw":12}, "srt":{"running":True,"enabled":True,"mode":"rgb","sent":10,"dropped":2},
          "system":{"cpu_percent":3.5,"current_rss_bytes":1048576}, "last_error":"camera lost"})
        await pilot.pause()
        text=app.query_one("#status").render().plain
        for value in ("DEGRADED","f0271544","1280x720","12","rgb","10","2","3.5","camera lost"): assert value in text
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
    assert client.commands == [("stop_gateway",{})]
  asyncio.run(scenario())
