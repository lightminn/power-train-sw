import threading
import time

import pytest

from powertrain_runtime.drive_service import ManualDriveService


def state(**changes):
    value = dict(push="ops_state", revision=1, authority_mode="IDLE",
                 chassis_mode="IDLE", gateway_state="DRIVE", gateway_input_fresh=True,
                 gateway_neutral=True, estop_latched=False, active_estop_sources=[],
                 wheels_stopped=True, field_age_s=dict(authority=0, gateway=0, safety=0, wheels=0))
    value.update(changes)
    return value


class Broker:
    def __init__(self, initial=None, reject=None):
        self.state = initial or state()
        self.actions = []
        self.reject = reject or {}
        self.responses = []
        self.push = True
        self.closed = False
        self.sock = object()

    def submit(self, action, **kwargs):
        request_id = str(len(self.actions))
        self.actions.append(action)
        status = self.reject.get(action, "FINAL_SUCCESS")
        if status == "FINAL_SUCCESS":
            if action == "clear_transient_hold":
                self.state.update(authority_mode="IDLE", gateway_state="DRIVE", gateway_input_fresh=True)
            if action == "authority_manual":
                self.state["authority_mode"] = "TELEOP"
            if action == "arm":
                self.state["chassis_mode"] = "ARMED"
            if action == "disarm":
                self.state["chassis_mode"] = "IDLE"
            if action == "authority_idle":
                self.state["authority_mode"] = "IDLE"
        self.responses.append(dict(request_id=request_id, status=status, detail=action))
        return request_id

    def pump(self):
        replies, self.responses = self.responses, []
        return replies + ([dict(self.state)] if self.push else [])

    def close(self):
        self.closed = True


def wait_for(test, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if test():
            return
        time.sleep(.005)
    assert test()


@pytest.fixture
def make_service():
    services = []
    def make(broker):
        service = ManualDriveService(lambda: broker, timeout_s=.3)
        services.append(service)
        wait_for(lambda: service.snapshot()["ops_state"] is not None)
        return service
    yield make
    for service in services:
        service.close()


def test_start_is_explicit_and_orders_hold_manual_arm(make_service):
    broker = Broker(state(authority_mode="MOTION_HOLD", gateway_state="MOTION_HOLD"))
    service = make_service(broker)
    assert broker.actions == []
    assert service.start()["status"] == "PENDING"
    wait_for(lambda: service.snapshot()["status"] == "SUCCEEDED")
    assert broker.actions == ["clear_transient_hold", "authority_manual", "arm"]
    assert "estop_reset" not in broker.actions


@pytest.mark.parametrize("change", [dict(estop_latched=True), dict(gateway_neutral=False),
    dict(wheels_stopped=False), dict(gateway_input_fresh=False),
    dict(field_age_s=dict(authority=0, gateway=0, safety=4, wheels=0)),
    dict(field_age_s=dict(authority=0, gateway=float("nan"), safety=0, wheels=0)),
    dict(chassis_mode="ESTOP"), dict(authority_mode="AUTONOMY")])
def test_invalid_start_never_sends_command(make_service, change):
    broker = Broker(state(**change))
    service = make_service(broker)
    service.start()
    wait_for(lambda: service.snapshot()["status"] == "REJECTED")
    assert broker.actions == []


@pytest.mark.parametrize("outcome", ["FINAL_REJECTED", "OUTCOME_UNKNOWN"])
def test_failed_manual_ack_never_arms(make_service, outcome):
    broker = Broker(reject={"authority_manual": outcome})
    service = make_service(broker)
    service.start()
    wait_for(lambda: service.snapshot()["status"] in ("REJECTED", "OUTCOME_UNKNOWN"))
    assert broker.actions == ["authority_manual"]


def test_stale_push_is_not_refreshed_by_polling(make_service):
    broker = Broker()
    service = make_service(broker)
    broker.push = False
    time.sleep(.55)
    service.start()
    wait_for(lambda: service.snapshot()["status"] == "REJECTED")
    assert broker.actions == []


def test_stop_cancels_start_wait_and_does_not_arm(make_service):
    broker = Broker(reject={"authority_manual": "PENDING"})
    service = make_service(broker)
    service.start()
    wait_for(lambda: "authority_manual" in broker.actions)
    assert service.stop()["status"] == "PENDING"
    wait_for(lambda: service.snapshot()["status"] == "SUCCEEDED")
    assert broker.actions == ["authority_manual", "disarm", "authority_idle"]


def test_disarm_failure_still_attempts_idle_but_never_reports_success(make_service):
    broker = Broker(reject={"disarm": "OUTCOME_UNKNOWN"})
    service = make_service(broker)
    service.stop()
    wait_for(lambda: service.snapshot()["status"] == "OUTCOME_UNKNOWN")
    assert broker.actions == ["disarm", "authority_idle"]


@pytest.mark.parametrize('chassis', ['IDLE', 'ESTOP'])
def test_stop_accepts_confirmed_stationary_hold_without_clearing_it(make_service, chassis):
    class HeldBroker(Broker):
        def submit(self, action, **kwargs):
            result = super().submit(action, **kwargs)
            self.state['chassis_mode'] = chassis
            return result
    broker = HeldBroker(state(chassis_mode=chassis, authority_mode='MOTION_HOLD',
                              estop_latched=chassis == 'ESTOP'),
                        reject={'authority_idle': 'FINAL_REJECTED'})
    service = make_service(broker)
    service.stop()
    wait_for(lambda: service.snapshot()['status'] != 'PENDING')
    assert service.snapshot()['status'] == 'SUCCEEDED'
    assert broker.actions == ['disarm', 'authority_idle']
    assert broker.state['authority_mode'] == 'MOTION_HOLD'
    assert broker.state['estop_latched'] is (chassis == 'ESTOP')


@pytest.mark.parametrize('authority', ['IDLE', 'MOTION_HOLD'])
@pytest.mark.parametrize('changes', [dict(wheels_stopped=False),
    dict(field_age_s=dict(authority=0, gateway=0, safety=0, wheels=1))])
def test_stop_never_confirms_moving_or_stale_wheels(make_service, authority, changes):
    broker = Broker(state(authority_mode=authority, **changes),
                    reject={'authority_idle': 'FINAL_REJECTED'} if authority == 'MOTION_HOLD' else {})
    service = make_service(broker)
    service.stop()
    wait_for(lambda: service.snapshot()['status'] != 'PENDING')
    assert service.snapshot()['status'] == 'OUTCOME_UNKNOWN'


@pytest.mark.parametrize('outcome', ['FINAL_REJECTED', 'OUTCOME_UNKNOWN'])
def test_confirmed_hold_does_not_hide_missing_disarm_ack(make_service, outcome):
    broker = Broker(state(authority_mode='MOTION_HOLD'),
                    reject={'disarm': outcome, 'authority_idle': 'FINAL_REJECTED'})
    service = make_service(broker)
    service.stop()
    wait_for(lambda: service.snapshot()['status'] != 'PENDING')
    assert service.snapshot()['status'] == 'OUTCOME_UNKNOWN'


def test_unconfirmed_arm_is_compensated_with_disarm_and_idle(make_service):
    broker = Broker(reject={"arm": "OUTCOME_UNKNOWN"})
    service = make_service(broker)
    service.start()
    wait_for(lambda: service.snapshot()["status"] == "OUTCOME_UNKNOWN")
    assert broker.actions == ["authority_manual", "arm", "disarm", "authority_idle"]


def test_stop_waits_for_prior_mutation_busy_response(make_service):
    broker = Broker(reject={"disarm": "FINAL_REJECTED"})
    service = make_service(broker)
    service.stop()
    wait_for(lambda: "disarm" in broker.actions)
    broker.reject.clear()
    wait_for(lambda: service.snapshot()["status"] == "SUCCEEDED")
    assert broker.actions.count("disarm") >= 2


def test_hold_can_clear_while_gateway_reports_gated_input_not_fresh(make_service):
    broker = Broker(state(authority_mode="MOTION_HOLD", gateway_state="MOTION_HOLD", gateway_input_fresh=False))
    service = make_service(broker)
    assert service.snapshot()["start_blocker"] is None
    service.start()
    wait_for(lambda: service.snapshot()["status"] == "SUCCEEDED")
    assert broker.actions == ["clear_transient_hold", "authority_manual", "arm"]


def test_ack_waits_for_delayed_fresh_state_instead_of_compensating(make_service):
    class DelayedBroker(Broker):
        after_arm = None
        def submit(self, action, **kwargs):
            result = super().submit(action, **kwargs)
            if action == "arm":
                self.after_arm = time.monotonic()
                self.state["field_age_s"] = dict(authority=.8, gateway=0, safety=.8, wheels=.8)
            return result
        def pump(self):
            if self.after_arm is not None and time.monotonic() - self.after_arm > .1:
                self.state["field_age_s"] = dict(authority=0, gateway=0, safety=0, wheels=0)
            return super().pump()
    broker = DelayedBroker()
    service = make_service(broker)
    service.start()
    wait_for(lambda: service.snapshot()["status"] != "PENDING")
    assert service.snapshot()["status"] == "SUCCEEDED"
    assert broker.actions == ["authority_manual", "arm"]


def test_no_new_frame_after_clear_never_advances_to_manual_or_arm(make_service):
    class NoNewFrameBroker(Broker):
        def submit(self, action, **kwargs):
            result = super().submit(action, **kwargs)
            if action == "clear_transient_hold":
                self.state.update(gateway_state="DISCONNECTED", gateway_input_fresh=False)
            return result
    broker = NoNewFrameBroker(state(authority_mode="MOTION_HOLD", gateway_state="MOTION_HOLD", gateway_input_fresh=False))
    service = make_service(broker)
    service.start()
    wait_for(lambda: service.snapshot()["status"] == "OUTCOME_UNKNOWN")
    assert broker.actions == ["clear_transient_hold"]


def test_fresh_estop_after_arm_ack_compensates_and_never_resets(make_service):
    class EstopBroker(Broker):
        def submit(self, action, **kwargs):
            result = super().submit(action, **kwargs)
            if action == "arm":
                self.state.update(estop_latched=True, chassis_mode="ESTOP")
            return result
    broker = EstopBroker()
    service = make_service(broker)
    service.start()
    wait_for(lambda: service.snapshot()["status"] == "REJECTED")
    assert broker.actions == ["authority_manual", "arm", "disarm", "authority_idle"]
    assert broker.state["estop_latched"] is True
