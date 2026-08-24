"""
Filename: hermes/revalexo/exo/state_machines/walking.py
Author: Elias Thiery <ethiery@vub.be>
Date: 2025-12-10
Version: 1.0
Description: Revalexo-specific state machine for the hierarchical control
    of the walking ambulation mode.
"""

from dataclasses import asdict
import numpy as np
from statemachine import Event, State, StateMachine

from hermes.utils.time_utils import get_time

from .base import ProsthesisStateMachine
from ..can_control.motor_cubemars import can_set_position_impedance
from ..utils.types import (
    ModeContext,
    PhaseEstimate,
    ServoImpedanceGains,
    ServoMotorEnum,
    ServoReference,
    StateEnum,
    StateTransition,
    WalkingFirstStrideEnum,
    WalkingParameters,
)


class Hurdle(StateMachine, ProsthesisStateMachine):
    # States.
    idle = State(
        value=StateEnum.Hurdle.IDLE.value,
        initial=True
    )
    walking = State(
        value=StateEnum.Walking.WALKING.value,
    )

    # Transitions.
    cycle = (
        idle.to(idle, unless="idle_to_walking")
        | idle.to(walking, cond="idle_to_walking")
        | walking.to(idle, cond="walking_to_idle")
        | walking.to(walking, unless="walking_to_idle")
    )

    def __init__(self, ctx: ModeContext):
        self._th_roll = [0]
        self._int = [0]
        self._vel = [0]
        self._phase = 0
        self._max_min = [0, 0, 0, 0]
        self._prev_th_roll = [0]
        self._prev_int = [0]
        self._prev_vel = [0]
        self._first_stride = WalkingFirstStrideEnum.NONE
        self._inactivity_timer = 0

        self._thigh_left_gyr = 0
        self._thigh_left_angle = 0
        self._thigh_left_roll = 0
        self._knee_left_roll = 0
        self._thigh_right_gyr = 0
        self._thigh_right_angle = 0
        self._thigh_right_roll = 0
        self._knee_right_roll = 0
        self._torso_roll = 0

        self._K = ctx.K
        self._bus = ctx.bus
        self._motor_latest_data = ctx.motor_latest_data
        self._fatigue = ctx.fatigue
        self._state_changed_queue = ctx.state_changed_queue
        self._phase_estimate_queue = ctx.phase_estimate_queue
        self._motor_command_queue = ctx.motor_command_queue

        # Personalized parameters.
        # TODO: source from `watchdog` observed YAML config file.
        self._param = WalkingParameters(
            inactivity_gyr_threshold=250,
            inactivity_idle_transition_time=2,
            traj_shift=[85, 75, -10, -20],
            inactivity_time_step=0.01,
            first_stride_end_gyr=-700,
            reset_phase_threshold=95,
            inactivity_angle_threshold=2,
        )

        # Positions for right hip, left hip, right knee and left knee for each phase percentage between 0 and 100 %.
        # TODO: source from a text file for ease and cleanliness.
        self._trajectory = [
            [12.8, 12.379220773930983, 12.012121209110303, 11.690259741251577, 11.479870128217069, 11.312987010121706, 11.212727274533817, 11.278181818993454, 11.393506495710575, 11.51688311886679, 11.587012989878293, 11.485714286155101, 11.247272734577455, 10.85454544237091, 10.196103903868273, 9.433766265505009, 8.591515174790786, 7.675151552355876, 6.72727276892987, 5.749350676571055, 4.837662353421519, 4.087878802409697, 3.442857162253061, 2.8818181941610375, 2.368181822810389, 1.912337661235621, 1.561688306178107, 1.2842424228286051, 1.0376623379929488, 0.8272727249584416, 0.5558441492319108, 0.28363635740509074, 0.021818179566544582, -0.32142857804081776, -0.7409091063376645, -1.2254545620276378, -1.6181818287854548, -2.063636376750651, -2.5467532600378484, -2.967532456892767, -3.388311673764752, -3.7848484752921236, -4.112121197590304, -4.405194794174398, -4.6642857086653065, -4.874675321699816, -5.072727267621819, -5.298701289884974, -5.579220773930984, -5.7948051934827465, -5.965454544459637, -6.09636363337891, -6.16818181741039, -6.2, -6.183116881133209, -6.042857139110204, -5.86363635970909, -5.618181813553244, -5.127272716472727, -4.480519456274578, -3.6909090532072724, -2.709090886312719, -1.3961038542263393, 0.08246756046439319, 1.6954544817415726, 3.4527272783825493, 5.300000018514302, 7.263636270503911, 9.296103822597795, 11.25194800731541, 13.10363634070111, 14.674545407732378, 16.14415580448238, 17.497402579769954, 18.619480515953992, 19.489090900136738, 20.224675313214107, 20.785714281306124, 21.29090909322338, 21.721212123503033, 22.048484845801216, 22.277272727272727, 22.458441560204825, 22.59870130222783, 22.530519477874584, 22.389090902296726, 22.184415575819664, 21.763636349750644, 21.214285712302036, 20.566666691146658, 19.84666665543466, 19.000649355012428, 18.105194843545817, 17.26363639140779, 16.467272719889444, 15.705194784556582, 15.003896123131721, 14.362337678207044, 13.731168839103521, 13.1],
            [-2.5, -1.097402579769944, 0.36424243809260604, 1.8876623287358072, 3.7811688460463833, 5.718181858296106, 7.669090961480727, 9.567272750810183, 11.462337692092765, 13.30519485280297, 14.988311757079039, 16.442857138448982, 17.707272707793454, 18.754545487010912, 19.624675315358076, 20.34415582299666, 20.904242412604606, 21.362424223822064, 21.76883115395362, 22.11168830540668, 22.322077918441188, 22.48242423951806, 22.592857145281634, 22.52272727427013, 22.35779220977588, 22.094805193482745, 21.67402596741373, 21.11454545136436, 20.47142857264081, 19.699999991514286, 18.80649348500371, 17.95090907221527, 17.165454538699635, 16.32857141270204, 15.555844133803337, 14.861818156958543, 14.272727256821819, 13.661038944177735, 13.076623369981075, 12.866233771553617, 12.511688326235248, 12.115151524707876, 11.787878802409695, 11.546103904369202, 11.357142860889795, 11.21688311886679, 11.25757575587394, 11.349350644942488, 11.489610386965492, 11.564935064494248, 11.534545455540362, 11.40363636662109, 10.99090909553766, 10.416883115781076, 9.698701286799253, 8.857142834661223, 7.963636345309083, 7.036363627106488, 6.054545432945455, 5.124675293156952, 4.293939368804848, 3.639393924208479, 3.0584415416905357, 2.5227272642064893, 2.0318182012090866, 1.6939393928921205, 1.3857142830693856, 1.1051948184994413, 0.8935065011105731, 0.6441558507680877, 0.3660606098831485, 0.10424243204460376, -0.2110389511205949, -0.5863636286493552, -1.077272725729872, -1.4945454500683681, -1.9090908990623414, -2.399999996142858, -2.835064937048612, -3.2454545482036377, -3.6381818149614595, -3.995454545454548, -4.316883120409651, -4.59740260445566, -4.808441566376253, -5.010909097703275, -5.210389616120224, -5.4909091001662365, -5.728571429232654, -5.924242419791517, -6.055151517193698, -6.146103895768275, -6.2, -6.2, -6.0945454533149075, -5.941558435366974, -5.731168836939516, -5.281818194161034, -4.701298710115024, -4.0],
            [-0.6, 0.5922078071955478, 1.8139394053808486, 3.068181811238961, 4.540909102480519, 6.100649383600373, 7.705454588811636, 9.27636365584291, 10.875324700671616, 12.420779258469016, 13.753246807687571, 14.799999996914288, 15.603636353896727, 16.127272743505454, 16.260389609613174, 16.17792208850167, 15.895757587395394, 15.437575776177937, 14.862337692092764, 14.161038981977734, 13.459740271862707, 12.829090922168726, 12.235714307534694, 11.604545468431168, 11.06818182281039, 10.594805193482745, 10.174025967413728, 9.722424239950058, 9.275324675985898, 8.854545449916884, 8.433766223847867, 8.025454536107635, 7.632727269349816, 7.214285706351019, 6.7935064802820015, 6.378787864976969, 6.051515142678788, 5.711688302320964, 5.384415579987384, 5.2441558477024115, 5.151948054372541, 5.167878780233698, 5.429696958072244, 5.859090889805197, 6.435714265106125, 7.2071428462326566, 8.221212093983038, 9.393506449424871, 10.796103869654921, 12.458441547861971, 14.249090894664736, 16.147272683994203, 18.518181791953264, 21.09415584845381, 23.862987055636015, 26.87857150913062, 29.890909151127296, 32.98441561747683, 36.49090916805195, 40.04935077299965, 43.550909219095274, 46.88909098653676, 50.24935074985679, 53.488961091303, 56.504545335429896, 58.96484849259831, 61.25714287697961, 63.36103886125419, 64.77662332592952, 65.82857141038777, 66.50848484752922, 66.57393939198884, 66.22012988798292, 65.4779220977588, 64.21558441955176, 62.623636383037066, 60.785714332991816, 58.47142858961224, 55.877922065358796, 53.157575737873934, 50.34303032610954, 47.12272727272725, 43.68051943467454, 40.03376614207641, 36.17857128310203, 32.31878774516894, 28.44415575819664, 24.236363497506446, 19.985714272269362, 15.82121224583753, 12.15575751857645, 8.548701315817977, 5.150649478486053, 2.345454638025956, 0.2709090730661565, -1.3922078231651276, -2.4441558153024188, -2.8389610319079805, -2.790909097080517, -2.3],
            [47.6, 50.96623380855213, 54.015757601651394, 56.82012985789722, 59.414935085322824, 61.7051948481744, 63.65454549067636, 64.9636363798691, 65.91428572640817, 66.51688311886679, 66.58701298987829, 66.08571428769795, 65.24181820373236, 64.06363632711272, 62.3298701500575, 60.29285723013877, 57.97696976014642, 55.489697070680236, 52.748052075998885, 49.716883215295354, 46.420779277754725, 43.05696977253042, 39.60714299048979, 35.750000084857135, 31.703246792258994, 27.548051934827452, 23.340259674137286, 19.309696948136228, 15.336363642535044, 11.40909086589091, 7.787012902706863, 4.636363574050907, 2.0181817956654453, -0.0642857526367433, -1.6162337992949953, -2.6169697080184253, -2.87878788585697, -2.7363636232493493, -2.167532429892763, -0.9753247054704934, 0.21688307566679654, 1.422424206110069, 2.6660605508431567, 4.077272669415589, 5.592857099767353, 7.205844133031918, 8.781818140974556, 10.367532416838603, 11.980519450103158, 13.333766225390727, 14.45818181121746, 15.374545433652376, 15.94545453928312, 16.23831168842189, 16.266233762266417, 15.985714278220408, 15.581818172654541, 15.083116876504635, 14.38181816638961, 13.680519456274578, 13.014545431924363, 12.425454531787633, 11.803246734401853, 11.222727264206489, 10.731818201209085, 10.332727271470544, 9.899999995371424, 9.409090932374022, 8.987013002221147, 8.566233776152131, 8.149090914824722, 7.756363648066905, 7.346753258655286, 6.925974032586267, 6.5051948065172525, 6.154545458276361, 5.8220779292411855, 5.471428574183673, 5.288311687650463, 5.175757575299394, 5.110303030839757, 5.336363636363638, 5.704545460716889, 6.195454557797406, 6.964285743379593, 7.891515187750796, 8.95194808060112, 10.354545500831184, 11.928571433861237, 13.651515086976993, 15.549696999308624, 17.767532456121348, 20.2331167584761, 22.968181727924694, 25.767272753729493, 28.696103991039724, 31.92207783359408, 35.387012898849754, 38.923376578413375, 42.5]
        ]

        # Kalman filter variables.
        self.L = 0.015  # prediction horizon / latency [s] (optional)
        self.wrap_threshold = 95  # detect wrap when drop >50%
        self.cycle_len = 100.0  # phase range per cycle (percent)
        # Tuning.
        self.Q = np.diag([1e-5, 1e-3])  # process noise: [phase, rate]
        self.R = 5e-3  # measurement noise variance
        self.P = np.diag([0.1, 1.0])  # initial covariance
        self.x = np.array([0.0, 1.0])  # initial state [phase_unwrapped, rate]
        self.cycles = 0
        self.prev_raw = 0.0

        super(Hurdle, self).__init__()

    # Post-transition synchronous callback.
    def after_transition(self, event: Event, state: State):
        # Stores when the state machine transitioned to a new gait phase.
        self._state_changed_queue.put(
            StateTransition(timestamp=get_time(), state=state.value)
        )
        # print(f"Completed transition to {state.name}", flush=True)

    def idle_to_walking(self):
        return self._vel[-1] > self._param.inactivity_gyr_threshold

    def walking_to_idle(self):
        return self._inactivity_timer > self._param.inactivity_idle_transition_time

    # Actions.
    def on_enter_idle(self):
        pass

    def on_enter_walking(self):
        # Phase angle based impedance/torque trajectory control.
        pass

    def _kalman_phase_update(self, phase_raw):
        dt = 0.01  # sample period [s]
        # matrices (constant velocity model)
        A = np.array([[1, dt], [0, 1]])
        H = np.array([[1, 0]])
        I = np.eye(2)

        # ---- 1. predict ----
        x_pred = A @ self.x
        P_pred = A @ self.P @ A.T + self.Q

        # ---- 2. unwrap ----
        n = int(round((x_pred[0] - phase_raw) / self.cycle_len))
        # if phase_raw < self.prev_raw - self.wrap_threshold:
        #     self.cycles += 1
        # self.prev_raw = phase_raw
        phase_unwrapped = phase_raw + self.cycle_len * n

        # ---- 3. update ----
        z = np.array([phase_unwrapped])
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T / S
        self.x = x_pred + (K @ y).ravel()
        self.P = (I - K @ H) @ P_pred

        # ---- 4. optional: latency compensation ----
        phi_pred = self.x[0] + self.x[1] * self.L

        # ---- 5. rewrap for output ----
        phi_out = phi_pred % self.cycle_len

        return phi_out, self.x[1]  # smoothed phase [0?100), and estimated rate

    def update_sensor_values(
        self,
        torso_angle: float = np.nan,
        thigh_left_angle: float = np.nan,
        thigh_right_angle: float = np.nan,
        thigh_left_roll: float = np.nan,
        thigh_right_roll: float = np.nan,
        knee_left_roll: float = np.nan,
        knee_right_roll: float = np.nan,
        thigh_left_gyr: int = 0,
        thigh_right_gyr: int = 0,
        dt: float = 0.01,
    ):
        self._thigh_left_gyr = thigh_left_gyr
        self._thigh_left_angle = thigh_left_angle
        self._thigh_left_roll = thigh_left_roll
        self._knee_left_roll = knee_left_roll
        self._thigh_right_gyr = thigh_right_gyr
        self._thigh_right_angle = thigh_right_angle
        self._thigh_right_roll = thigh_right_roll
        self._knee_right_roll = knee_right_roll

        # Check if sensor values remain more or less constant.
        if abs(thigh_right_gyr) < self._param.inactivity_gyr_threshold:
            self._inactivity_timer += self._param.inactivity_time_step
        else:
            self._inactivity_timer = 0

        if (
            self._first_stride == WalkingFirstStrideEnum.NONE
            and thigh_right_gyr > self._param.inactivity_gyr_threshold
        ):
            self._first_stride = WalkingFirstStrideEnum.ONGOING
        elif self._first_stride == WalkingFirstStrideEnum.ONGOING:
            if thigh_right_gyr < self._param.first_stride_end_gyr:
                self._first_stride = WalkingFirstStrideEnum.TAKEN

        # Using integral.
        # self.th_roll.append(th_roll)
        # self.int.append(self.int[-1] + self.th_roll[-1]*dt)

        # Gamma = -(self.max_min[1] + self.max_min[3])/2
        # gamma = -(self.max_min[0] + self.max_min[2])/2
        # z = np.abs(self.max_min[0] - self.max_min[2])/np.abs(self.max_min[1] - self.max_min[3])

        # self.phase = (np.arctan2((self.int[-1]+Gamma)*z,self.th_roll[-1]+gamma) + np.pi)/(2*np.pi)*100

        # print(f"{Gamma}, {gamma}, {z}, {self.phase}")

        # if self.phase >= 95 or self.first_stride == WalkingFirstStrideEnum.TAKEN:
        #     self.first_stride = WalkingFirstStrideEnum.DEACTIVATED
        #     self.prev_th_roll = self.th_roll
        #     self.prev_int = self.int
        #     self.th_roll = [0]
        #     self.int = [0]
        #     self.max_min = [max(self.prev_th_roll), max(self.prev_int), min(self.prev_th_roll), min(self.prev_int)]

        # Using velocity.
        self._th_roll.append(thigh_right_angle)
        self._vel.append(thigh_right_gyr)

        Gamma = -(self._max_min[1] + self._max_min[3]) / 2
        gamma = -(self._max_min[0] + self._max_min[2]) / 2

        if self._max_min[1] - self._max_min[3] != 0:
            z = np.abs(self._max_min[0] - self._max_min[2]) / np.abs(
                self._max_min[1] - self._max_min[3]
            )
        else:
            z = 0

        phase, _ = self._kalman_phase_update(
            (np.arctan2((self._vel[-1] + Gamma) * z, self._th_roll[-1] + gamma) + np.pi)
            / (2 * np.pi)
            * 100
        )
        if (
            self._first_stride == WalkingFirstStrideEnum.TAKEN
            or self._first_stride == WalkingFirstStrideEnum.DEACTIVATED
        ):
            self._phase = phase
        else:
            self._phase = (
                (
                    np.arctan2((self._vel[-1] + Gamma) * z, self._th_roll[-1] + gamma)
                    + np.pi
                )
                / (2 * np.pi)
                * 100
            )

        # Push `phase` estimate into the upstream queue.
        self._phase_estimate_queue.put(
            PhaseEstimate(timestamp=get_time(), phase=self._phase)
        )

        if self._first_stride == WalkingFirstStrideEnum.TAKEN or (
            self._first_stride == WalkingFirstStrideEnum.DEACTIVATED
            and (
                self._phase >= self._param.reset_phase_threshold
                or abs(thigh_right_roll - max(self._prev_th_roll))
                <= self._param.inactivity_angle_threshold
            )
        ):
            self._first_stride = WalkingFirstStrideEnum.DEACTIVATED
            self._prev_th_roll = self._th_roll
            self._prev_vel = self._vel
            self._th_roll = [0]
            self._vel = [0]

            # if not self._prev_th_roll or not self._prev_vel:
            #     print("empty", flush=True)
            #     pass

            self._max_min = [
                max(self._prev_th_roll),
                max(self._prev_vel),
                min(self._prev_th_roll),
                min(self._prev_vel),
            ]

        # print(f"th_roll: {self._th_roll[-1]:.2f}, th_gyr: {self._vel[-1]:.2f}, kn_roll: {knee_right_roll:.2f}, phase: {self._phase:.2f}")

    def step(self) -> None:
        self.send("cycle")

    def send_data(self) -> dict:
        shift = self._param.traj_shift
        data = {
            "Lth_roll": self._thigh_left_angle,
            "Rth_roll": self._thigh_right_angle,
            "Lkn_roll": self._knee_left_roll,
            "Rkn_roll": self._knee_right_roll,
            "Rth_gyr": self._thigh_right_gyr,
            "phase": self._phase,
            "Lth_traj": float(
                -self._trajectory[0][(min(int(self._phase), 99) + shift[0]) % 100]
            ),
            "Rth_traj": float(
                -self._trajectory[1][(min(int(self._phase), 99) + shift[1]) % 100]
            ),
            "Lsh_traj": float(
                self._trajectory[2][(min(int(self._phase), 99) + shift[2]) % 100]
            ),
            "Rsh_traj": float(
                self._trajectory[3][(min(int(self._phase), 99) + shift[3]) % 100]
            ),
        }
        return data
