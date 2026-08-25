"""
Filename: hermes/revalexo/exo/controller/mode_selection.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2025-12-10
Version: 1.0
Description: Inter-ambulation mode state machine controller that implements
    how transitions between activities are done and what the exoskeleton does.
"""

from statemachine import Event, StateMachine, State

from hermes.utils.time_utils import get_time

from ..utils.types import ModeContext, ModeEnum, ModeTransition
from ..state_machines import (
    ExoStateMachine,
    Idle,
    Walking,
    SitToStand,
    StairAscent,
    StairDescent,
)


class ModeSelectionMachine(StateMachine):
    """High-level controller for locomotion mode control.

    Asynchronously generates telemetry data about locomotion mode,
    mid-level controller state, and gait phase.
    The locomotion mode data from the exo indicates when the exo controller actually switched
    to the new mode, based on the user-defined safe transitions definition, in response to the
    upstream (AI) locomotion intent prediction. To correlate to the specific prediction in post,
    a sequence ID of the prediction is tracked and referenced by the controller.
    All the data is pushed upstream to the parent `ExoPipeline` that manages the lifecycle of the system,
    live communication across HERMES hosts, and secure data collection.

    Fatigue estimation is directly passed to the current `ExoStateMachine`
    and will take effect during the next loop evaluation (100Hz).

    Inter-mode transition instantiates the mid-level state machine from scratch
    to ensure the current implementation always starts an activity from the initial state.
    This behavior may require changes for fluent locomotion mode switching (i.e. transition
    into non-initial phase of the gait cycle of the next mode).
    This will also offload resources, tighten loop timings, and make realtimeness more stable.
    """

    idle = State(
        name=ModeEnum.IDLE.value.text,
        value=ModeEnum.IDLE.value.id,
        initial=True,
    )
    walking = State(
        name=ModeEnum.WALKING.value.text,
        value=ModeEnum.WALKING.value.id,
    )
    sit_to_stand = State(
        name=ModeEnum.SIT_TO_STAND.value.text,
        value=ModeEnum.SIT_TO_STAND.value.id,
    )
    stair_ascent = State(
        name=ModeEnum.STAIR_ASCENT.value.text,
        value=ModeEnum.STAIR_ASCENT.value.id,
    )
    stair_descent = State(
        name=ModeEnum.STAIR_DESCENT.value.text,
        value=ModeEnum.STAIR_DESCENT.value.id,
    )

    allow_event_without_transition: bool = True
    _exo_mode: ExoStateMachine
    _sequence_id: int
    _source: int

    # NOTE: if safety of switching from Any to Target state (especially for larger #states) can be generalized,
    #   declare target-centric transitions with (`from_`)[https://python-statemachine.readthedocs.io/en/latest/transitions.html#from-and-from-any]

    # From any to `idle` ambulation mode must be immediate.
    to_idle = idle.from_.any()

    # NOTE: combination of conditions is possible to reuse shared logic among transitions
    #   [https://python-statemachine.readthedocs.io/en/latest/guards.html#condition-expressions]

    # From any to `walking`.
    to_walking = (
        idle.to(walking, cond="is_safe_idle_to_walking")
        | sit_to_stand.to(walking, cond="is_safe_sit_to_stand_to_walking")
        | stair_ascent.to(walking, cond="is_safe_stair_ascent_to_walking")
        | stair_descent.to(walking, cond="is_safe_stair_descent_to_walking")
    )

    # From any to `sit_to_stand`.
    to_sit_to_stand = (
        idle.to(sit_to_stand, cond="is_safe_idle_to_sit_to_stand")
        | walking.to(sit_to_stand, cond="is_safe_walking_to_sit_to_stand")
        | stair_ascent.to(sit_to_stand, cond="is_safe_stair_ascent_to_sit_to_stand")
        | stair_descent.to(sit_to_stand, cond="is_safe_stair_descent_to_sit_to_stand")
    )

    # From any to `stair_ascent`.
    to_stair_ascent = (
        idle.to(stair_ascent, cond="is_safe_idle_to_stair_ascent")
        | walking.to(stair_ascent, cond="is_safe_walking_to_stair_ascent")
        | sit_to_stand.to(stair_ascent, cond="is_safe_sit_to_stand_to_stair_ascent")
        | stair_descent.to(stair_ascent, cond="is_safe_stair_descent_to_stair_ascent")
    )

    # From any to `stair_descent`.
    to_stair_descent = (
        idle.to(stair_descent, cond="is_safe_idle_to_stair_descent")
        | walking.to(stair_descent, cond="is_safe_walking_to_stair_descent")
        | sit_to_stand.to(stair_descent, cond="is_safe_sit_to_stand_to_stair_descent")
        | stair_ascent.to(stair_descent, cond="is_safe_stair_ascent_to_stair_descent")
    )

    def __init__(self, ctx: ModeContext, is_immediate_mode_switch: bool = False):
        self._ctx = ctx
        self._is_immediate_mode_switch = is_immediate_mode_switch
        super(ModeSelectionMachine, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the exo transitioned to a new locomotion mode, and based on which `sequence_id` from upstream LMR.
        self._ctx.mode_changed_queue.put(
            ModeTransition(
                timestamp=get_time(), mode=state.value, sequence_id=self._sequence_id, source=self._source,
            )
        )
        print(f"[Prosthesis] Completed transition to {state.name}", flush=True)

    # TODO: Internal checks evaluated on each upstream state transition request to the operation mode FSM (for safe transitions).
    def is_safe_idle_to_walking(self) -> bool:
        return True or self._is_immediate_mode_switch

    def is_safe_sit_to_stand_to_walking(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_stair_ascent_to_walking(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_stair_descent_to_walking(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_idle_to_sit_to_stand(self) -> bool:
        return True or self._is_immediate_mode_switch

    def is_safe_walking_to_sit_to_stand(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_stair_ascent_to_sit_to_stand(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_stair_descent_to_sit_to_stand(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_idle_to_stair_ascent(self) -> bool:
        return True or self._is_immediate_mode_switch

    def is_safe_walking_to_stair_ascent(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_sit_to_stand_to_stair_ascent(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_stair_descent_to_stair_ascent(self) -> bool:
        return False or self._is_immediate_mode_switch

    def is_safe_idle_to_stair_descent(self) -> bool:
        return True or self._is_immediate_mode_switch

    def is_safe_walking_to_stair_descent(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_sit_to_stand_to_stair_descent(self) -> bool:
        return self._exo_mode.is_safe_to_switch() or self._is_immediate_mode_switch

    def is_safe_stair_ascent_to_stair_descent(self) -> bool:
        return False or self._is_immediate_mode_switch

    # Transition callbacks.
    # TODO: `watchdog` based `ConfigManager` will break (not live update config from files anymore) if a state machine is not instantiated on every transition.
    def on_enter_idle(self):
        self._exo_mode = Idle(self._ctx)

    def on_exit_idle(self):
        pass

    def on_enter_walking(self):
        self._exo_mode = Walking(self._ctx)

    def on_exit_walking(self):
        pass

    def on_enter_sit_to_stand(self):
        self._exo_mode = SitToStand(self._ctx)

    def on_exit_sit_to_stand(self):
        pass

    def on_enter_stair_ascent(self):
        self._exo_mode = StairAscent(self._ctx)

    def on_exit_stair_ascent(self):
        pass

    def on_enter_stair_descent(self):
        self._exo_mode = StairDescent(self._ctx)

    def on_exit_stair_descent(self):
        pass

    # Wrappers for abstract exo state machine controls.
    def update_sensor_values(self, **kwargs) -> None:
        self._exo_mode.update_sensor_values(**kwargs)

    def step(self) -> None:
        self._exo_mode.step()

    def send_data(self):
        return self._exo_mode.send_data()
