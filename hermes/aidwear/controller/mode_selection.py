from statemachine import Event, StateMachine, State

from hermes.utils.time_utils import get_time

from ..utils.types import (
    ModeContext,
    ModeEnum,
    ModeTransition
)
from ..state_machines import (
    ProsthesisStateMachine,
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
    The locomotion mode data from the prosthesis indicates when the prosthesis controller actually switched
    to the new mode, based on the user-defined safe transitions definition, in response to the
    upstream (AI) locomotion intent prediction. To correlate to the specific prediction in post,
    a sequence ID of the prediction is tracked and referenced by the controller.
    All the data is pushed upstream to the parent `ProsthesisPipeline` that manages the lifecycle of the system,
    live communication across HERMES hosts, and secure data collection.

    Fatigue estimation is directly passed to the current `ProsthesisStateMachine`
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

    _prosthesis_mode: ProsthesisStateMachine
    _sequence_id: int

    # From any to `idle`.
    to_idle = (
        walking.to(idle, cond="is_safe_walking_to_idle")
        | sit_to_stand.to(idle, cond="is_safe_sit_to_stand_to_idle")
        | stair_ascent.to(idle, cond="is_safe_stair_ascent_to_idle")
        | stair_descent.to(idle, cond="is_safe_stair_descent_to_idle")
    )

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

    def __init__(self, ctx: ModeContext):
        self._ctx = ctx
        super(ModeSelectionMachine, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the prosthesis transitioned to a new locomotion mode, and based on which `sequence_id` from upstream LMR.
        self._ctx.mode_changed_queue.put(
            ModeTransition(
                timestamp=get_time(), mode=state.value, sequence_id=self._sequence_id
            )
        )
        print(f"Completed transition to {state.name}", flush=True)

    # TODO: Internal checks evaluated on each upstream state transition request to the operation mode FSM (for safe transitions).
    def is_safe_walking_to_idle(self) -> bool:
        return True

    def is_safe_sit_to_stand_to_idle(self) -> bool:
        return True

    def is_safe_stair_ascent_to_idle(self) -> bool:
        return True

    def is_safe_stair_descent_to_idle(self) -> bool:
        return True

    def is_safe_idle_to_walking(self) -> bool:
        return True

    def is_safe_sit_to_stand_to_walking(self) -> bool:
        return True

    def is_safe_stair_ascent_to_walking(self) -> bool:
        return True

    def is_safe_stair_descent_to_walking(self) -> bool:
        return True

    def is_safe_idle_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_walking_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_stair_ascent_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_stair_descent_to_sit_to_stand(self) -> bool:
        return True

    def is_safe_idle_to_stair_ascent(self) -> bool:
        return True

    def is_safe_walking_to_stair_ascent(self) -> bool:
        return True

    def is_safe_sit_to_stand_to_stair_ascent(self) -> bool:
        return True

    def is_safe_stair_descent_to_stair_ascent(self) -> bool:
        return True

    def is_safe_idle_to_stair_descent(self) -> bool:
        return True

    def is_safe_walking_to_stair_descent(self) -> bool:
        return True

    def is_safe_sit_to_stand_to_stair_descent(self) -> bool:
        return True

    def is_safe_stair_ascent_to_stair_descent(self) -> bool:
        return True

    # Transition callbacks.
    def on_enter_idle(self):
        self._prosthesis_mode = Idle(self._ctx)

    def on_exit_idle(self):
        pass

    def on_enter_walking(self):
        self._prosthesis_mode = Walking(self._ctx)

    def on_exit_walking(self):
        pass

    def on_enter_sit_to_stand(self):
        self._prosthesis_mode = SitToStand(self._ctx)

    def on_exit_sit_to_stand(self):
        pass

    def on_enter_stair_ascent(self):
        self._prosthesis_mode = StairAscent(self._ctx)

    def on_exit_stair_ascent(self):
        pass

    def on_enter_stair_descent(self):
        self._prosthesis_mode = StairDescent(self._ctx)

    def on_exit_stair_descent(self):
        pass

    # Wrappers for abstract prosthesis state machine controls.
    def update_sensor_values(self, **kwargs) -> None:
        self._prosthesis_mode.update_sensor_values(**kwargs)

    def step(self) -> None:
        self._prosthesis_mode.step()

    def send_data(self):
        return self._prosthesis_mode.send_data()
