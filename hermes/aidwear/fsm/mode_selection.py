from collections import deque
from multiprocessing import Queue
from multiprocessing.sharedctypes import Synchronized
import can
from statemachine import Event, StateMachine, State

from hermes.utils.time_utils import get_time

from ..utils.types import ModeStateEnum, ModeTransition, PhaseTransition, ServoImpedanceGains, ServoMotorState
from .state_machines import (
    CyberlegStateMachine,
    Walking,
    SitToStand,
    StairDescent,
    StairAscent,
)


class ModeSelectionMachine(StateMachine):
    # Define states
    walking = State(
        name=ModeStateEnum.WALKING.value.text,
        value=ModeStateEnum.WALKING.value.id,
        initial=True,
    )
    sit_to_stand = State(
        name=ModeStateEnum.SIT_TO_STAND.value.text,
        value=ModeStateEnum.SIT_TO_STAND.value.id,
    )
    stair_ascent = State(
        name=ModeStateEnum.STAIR_ASCENT.value.text,
        value=ModeStateEnum.STAIR_ASCENT.value.id,
    )
    stair_descent = State(
        name=ModeStateEnum.STAIR_DESCENT.value.text,
        value=ModeStateEnum.STAIR_DESCENT.value.id,
    )

    _exo_mode: CyberlegStateMachine
    _sequence_id: int

    # From walking
    to_walking = (
        sit_to_stand.to(walking, cond="is_safe_sit_to_stand_to_walking")
        | stair_ascent.to(walking, cond="is_safe_stair_ascent_to_walking")
        | stair_descent.to(walking, cond="is_safe_stair_descent_to_walking")
    )

    # From any to `sit_to_stand`
    to_sit_to_stand = (
        walking.to(sit_to_stand, cond="is_safe_walking_to_sit_to_stand")
        | stair_ascent.to(sit_to_stand, cond="is_safe_stair_ascent_to_sit_to_stand")
        | stair_descent.to(sit_to_stand, cond="is_safe_stair_descent_to_sit_to_stand")
    )

    # From any to `stair_ascent`
    to_stair_ascent = (
        walking.to(stair_ascent, cond="is_safe_walking_to_stair_ascent")
        | sit_to_stand.to(stair_ascent, cond="is_safe_sit_to_stand_to_stair_ascent")
        | stair_descent.to(stair_ascent, cond="is_safe_stair_descent_to_stair_ascent")
    )

    # From any to `stair_descent`
    to_stair_descent = (
        walking.to(stair_descent, cond="is_safe_walking_to_stair_descent")
        | sit_to_stand.to(stair_descent, cond="is_safe_sit_to_stand_to_stair_descent")
        | stair_ascent.to(stair_descent, cond="is_safe_stair_ascent_to_stair_descent")
    )

    def __init__(
        self,
        bus: can.BusABC,
        K: dict[str, ServoImpedanceGains],
        motor_latest_state: dict[int, deque[ServoMotorState | None]],
        next_fatigue: Synchronized[float],
        mode_changed_queue: "Queue[ModeTransition]",
        phase_changed_queue: "Queue[PhaseTransition]",
    ):
        self._K = K
        self._bus = bus
        self._motor_latest_state = motor_latest_state
        # NOTE: fatigue is directly passed to the enclosed `ExoStateMachine`
        #       and will take effect during the next loop evaluation (100Hz).
        #       So no need to record time support level, will be recorded by upstream Storage.
        self._fatigue = next_fatigue
        self._mode_changed_queue = mode_changed_queue
        self._phase_changed_queue = phase_changed_queue
        super(ModeSelectionMachine, self).__init__()

    # Post-transition synchronous callback
    def after_transition(self, event: Event, state: State):
        # Stores when the exo transitioned to a new locomotion mode, and based on which `sequence_id` from upstream LMR.
        self._mode_changed_queue.put(ModeTransition(timestamp=get_time(), mode=state.value, sequence_id=self._sequence_id))
        print(f"Completed transition to {state.name}", flush=True)

    # TODO: Internal checks evaluated on each upstream state transition request to the operation mode FSM (for safe transitions)
    def is_safe_sit_to_stand_to_walking(self) -> bool:
        return True

    def is_safe_stair_ascent_to_walking(self) -> bool:
        return True

    def is_safe_stair_descent_to_walking(self) -> bool:
        return True

    def is_safe_walking_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_stair_ascent_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_stair_descent_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_walking_to_stair_ascent(self) -> bool:
        return True

    def is_safe_sit_to_stand_to_stair_ascent(self) -> bool:
        return True

    def is_safe_stair_descent_to_stair_ascent(self) -> bool:
        return True

    def is_safe_walking_to_stair_descent(self) -> bool:
        return True

    def is_safe_sit_to_stand_to_stair_descent(self) -> bool:
        return True

    def is_safe_stair_ascent_to_stair_descent(self) -> bool:
        return True

    # Transition callbacks
    def on_enter_walking(self):
        self._exo_mode = Walking(self._bus, self._K, self._motor_latest_state, self._fatigue, self._phase_changed_queue)

    def on_exit_walking(self):
        pass

    def on_enter_sit_to_stand(self):
        self._exo_mode = SitToStand(self._bus, self._K, self._motor_latest_state, self._fatigue, self._phase_changed_queue)

    def on_exit_sit_to_stand(self):
        pass

    def on_enter_stair_ascent(self):
        self._exo_mode = StairAscent(self._bus, self._K, self._motor_latest_state, self._fatigue, self._phase_changed_queue)

    def on_exit_stair_ascent(self):
        pass

    def on_enter_stair_descent(self):
        self._exo_mode = StairDescent(self._bus, self._K, self._motor_latest_state, self._fatigue, self._phase_changed_queue)

    def on_exit_stair_descent(self):
        pass

    # Wrappers for abstract exo state machine controls
    def update_sensor_values(self, **kwargs) -> None:
        self._exo_mode.update_sensor_values(**kwargs)

    def step(self) -> None:
        self._exo_mode.step()

    def send_data(self):
        return self._exo_mode.send_data()
