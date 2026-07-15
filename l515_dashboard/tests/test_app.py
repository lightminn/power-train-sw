import asyncio
import os
import socket
import time
import uuid

import pytest

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


def test_observability_renders_ten_node_can_matrix_and_consistency_warns():
  async def scenario():
    app=DashboardApp(Client(),poll_interval_s=60)
    payload={
      "run_id":"run-can", "drop_count":0, "health":{"status":"OK"},
      "recent_events":{"CAN_HEALTH":{"payload":{
        "ak_nodes":[
          {"can_id":1,"physical_wheel":"front_left","last_feedback_age_ms":12.5,
           "feedback_rate_hz":49.8,"steer_fault":0,"stale":False,"recovery_count":1},
          {"can_id":2,"physical_wheel":"front_right","last_feedback_age_ms":13.0,
           "feedback_rate_hz":49.7,"steer_fault":0,"stale":False,"recovery_count":0},
          {"can_id":3,"physical_wheel":"rear_left","last_feedback_age_ms":400.0,
           "feedback_rate_hz":0.0,"steer_fault":7,"stale":True,"recovery_count":2},
          {"can_id":4,"physical_wheel":"rear_right","last_feedback_age_ms":15.0,
           "feedback_rate_hz":49.9,"steer_fault":0,"stale":False,"recovery_count":0},
        ],
        "odrive_nodes":[
          {"node_id":node,"physical_wheel":wheel,"last_heartbeat_age_ms":20.0,
           "last_encoder_age_ms":18.0,"axis_state":8,"axis_error":0,
           "stale":False,"recovery_count":0}
          for node,wheel in zip(range(11,17),(
            "front_left","front_right","mid_left","mid_right","rear_left","rear_right"))
        ],
        "bus":{"rx_packet_delta":120,"tx_packet_delta":80,"error_warning":True,
               "error_passive":False,"bus_off_delta":1,"restart_count":3},
        "owner":{"pid":4321,"process_name":"chassis_node",
                 "lock_path":"/run/powertrain/can0.lock",
                 "acquisition_time":"2026-07-15T00:00:00+00:00"},
        "interlock":{"motion_hold_sources":["robot_arm"],
                     "latched_estop_sources":[],"reset_required":False},
        "wheel_consistency":{"warnings":[
          {"severity":"WARN","code":"same_side_delta","wheels":["rear_left"],
           "value":0.6,"threshold":0.25}],"terrain_speed_cap":0.4,
          "wheel_yaw_rate_rad_s":0.1,"imu_yaw_rate_rad_s":0.0},
      }}},
      "channel_health":{},
    }
    async with app.run_test() as pilot:
      app.show_observability_status(payload); await pilot.pause()
      text=app.query_one("#observability-status").render().plain
      for value in (
        "AK1 front_left","AK4 rear_right","OD11 front_left","OD16 rear_right",
        "rxΔ=120","txΔ=80","bus-offΔ=1","chassis_node","4321",
        "robot_arm","same_side_delta","speed cap=0.4",
      ):
        assert value in text
  asyncio.run(scenario())


def test_observability_arm_row_renders_raw_status_mission_and_hold_reason():
  from powertrain_observability.arm_adapter import (
    ArmObservation,
    build_arm_events,
  )

  async def scenario():
    observation=ArmObservation(
      raw_status="FAILED", source_mission_id=12,
      stamp_sec=123, stamp_nanosec=456_000_000, accepted=True,
      contract_violation=False,
      current_mission_id=41, arm_posture="STOWED_LOCKED",
      hold_reason="arm_failure:FAILED",
      source_detail="state=FAILED_HOLD;operation=PICKUP",
    )
    event,=build_arm_events(observation)
    app=DashboardApp(Client(),poll_interval_s=60)
    payload={
      "run_id":"run-arm", "drop_count":0, "health":{"status":"OK"},
      "recent_events":{"ARM_RESULT":event}, "channel_health":{},
    }
    async with app.run_test() as pilot:
      app.show_observability_status(payload); await pilot.pause()
      text=app.query_one("#observability-status").render().plain
      for value in (
        "FAILED","stamp=123.456000000","mission_id=41","arm_failure:FAILED",
      ):
        assert value in text
  asyncio.run(scenario())


def test_arm_fixtures_cross_real_daemon_socket_and_reach_tui(tmp_path):
  from powertrain_observability.arm_adapter import (
    ArmObservation,
    build_arm_events,
  )
  from powertrain_observability.client import EventClient, ObservabilityClient
  from powertrain_observability.server import ObservabilityServer

  probe=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
  try:
    probe.setsockopt(socket.SOL_SOCKET,socket.SO_PASSCRED,1)
    probe.bind("\0test-t5-app-probe-"+uuid.uuid4().hex)
  except PermissionError as exc:
    pytest.skip(f"sandbox blocks AF_UNIX abstract sockets/SO_PASSCRED: {exc}")
  finally:
    probe.close()

  def wait_for(predicate,timeout=2.0):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
      value=predicate()
      if value: return value
      time.sleep(0.01)
    return predicate()

  fixtures=(
    ("FUTURE_ARM_STATUS",False,"arm_contract_violation:FUTURE_ARM_STATUS"),
    ("FAILED",True,"arm_failure:FAILED"),
    ("GRIP_LOST",True,"grip_lost_latched"),
  )
  for index,(raw_status,accepted,hold_reason) in enumerate(fixtures):
    suffix=f"{os.getpid()}-{index}-{uuid.uuid4().hex}"
    event_socket=f"@test-t5-app-events-{suffix}"
    status_socket=f"@test-t5-app-status-{suffix}"
    server=ObservabilityServer(
      event_socket=event_socket,status_socket=status_socket,
      lock_path=tmp_path/f"observability-{index}.lock",
      run_directory=tmp_path/f"runs-{index}",run_id=f"task5-app-{index}",
    )
    server.start()
    try:
      observation=ArmObservation(
        raw_status=raw_status,source_mission_id=12,
        stamp_sec=123,stamp_nanosec=456_000_000,accepted=accepted,
        contract_violation=not accepted,
        current_mission_id=41,arm_posture="STOWED_LOCKED",
        hold_reason=hold_reason,
        source_detail="state=FAILED_HOLD;operation=PICKUP",
      )
      producer=EventClient(event_socket)
      assert all(producer.emit(event) for event in build_arm_events(observation))
      consumer=ObservabilityClient(status_socket,request_timeout_s=0.2)

      def received():
        snapshot=consumer.poll()
        if snapshot is None: return None
        recent=snapshot.payload["recent_events"]
        if "ARM_RESULT" not in recent: return None
        if not accepted and "CONTRACT_VIOLATION" not in recent: return None
        return snapshot

      observed=wait_for(received)
      assert observed is not None
      if not accepted:
        violation=observed.payload["recent_events"]["CONTRACT_VIOLATION"]
        assert violation["payload"]["raw_status"]==raw_status
        assert violation["payload"]["stamp"]=={
          "sec":123,"nanosec":456_000_000,
        }

      async def scenario():
        app=DashboardApp(
          Client(),observability_client=consumer,poll_interval_s=60,
        )
        async with app.run_test() as pilot:
          app.refresh_status(); await pilot.pause()
          text=app.query_one("#observability-status").render().plain
          for value in (
            raw_status,"stamp=123.456000000","mission_id=41",hold_reason,
          ):
            assert value in text
      asyncio.run(scenario())
    finally:
      server.stop()
