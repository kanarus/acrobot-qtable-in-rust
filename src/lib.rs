mod np;

use ::oxide_control::physics::{ObjectId, joint, obj};
use ::qtable::{QConfig, QTable, QUpdate};
use ::std::f64::consts::PI;

pub struct Acrobot {
    raw: oxide_control::RawPhysics,
    elbow_id: ObjectId<joint::Hinge>,
    shoulder_id: ObjectId<joint::Hinge>,
    actuator_id: ObjectId<obj::Actuator>,
}
impl Acrobot {
    pub fn new() -> Self {
        let raw = oxide_control::RawPhysics::from_xml("acrobot.xml").unwrap();
        let elbow_id = raw.object_id::<joint::Hinge>("elbow").unwrap();
        let shoulder_id = raw.object_id::<joint::Hinge>("shoulder").unwrap();
        let actuator_id = raw.object_id::<obj::Actuator>("shoulder").unwrap();
        Self {
            raw,
            elbow_id,
            shoulder_id,
            actuator_id,
        }
    }
}
impl std::ops::Deref for Acrobot {
    type Target = oxide_control::RawPhysics;

    fn deref(&self) -> &Self::Target {
        &self.raw
    }
}
impl std::ops::DerefMut for Acrobot {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.raw
    }
}
impl oxide_control::Physics for Acrobot {}

pub struct Orientation {
    sin: f64,
    cos: f64,
}
impl Orientation {
    pub fn from_rad(rad: f64) -> Self {
        Self {
            sin: rad.sin(),
            cos: rad.cos(),
        }
    }

    pub fn to_rad(&self) -> f64 {
        f64::atan2(self.sin, self.cos)
    }
}

pub struct AcrobotObservation {
    pub elbow_orientation: Orientation,
    pub shoulder_orientation: Orientation,
    pub elbow_velocity: f64,
    pub shoulder_velocity: f64,
    __: ()
}
impl oxide_control::Observation for AcrobotObservation {
    type Physics = Acrobot;

    fn generate(physics: &Self::Physics) -> Self {
        let [elbow_rad] = physics.qpos(physics.elbow_id);
        let [shoulder_rad] = physics.qpos(physics.shoulder_id);
        let [elbow_velocity] = physics.qvel(physics.elbow_id);
        let [shoulder_velocity] = physics.qvel(physics.shoulder_id);
        Self {
            elbow_orientation: Orientation::from_rad(elbow_rad),
            shoulder_orientation: Orientation::from_rad(shoulder_rad),
            elbow_velocity,
            shoulder_velocity,
            __: (),
        }
    }
}

#[derive(Clone, Copy)]
pub struct AcrobotState(AcrobotStateInit);
#[derive(Clone, Copy)]
pub struct AcrobotStateInit {
    pub arm_rad: f64,
    pub arm_vel: f64,
    pub pendulum_rad: f64,
    pub pendulum_vel: f64,
    pub n_arm_digitization: usize,
    pub n_pendulum_digitization: usize,
}
impl AcrobotState {
    pub fn new(init: AcrobotStateInit) -> Self {
        Self(init)
    }
    
    pub fn n_arm_rad(&self) -> usize {
        np::digitize(self.0.arm_rad, &np::linspace(-0.9 * PI, 0.9 * PI, self.0.n_arm_digitization + 1)[1..self.0.n_arm_digitization])
    }
    pub fn n_pendulum_rad(&self) -> usize {
        np::digitize(self.0.pendulum_rad, &np::linspace(-0.2 * PI, 0.2 * PI, self.0.n_pendulum_digitization + 1)[1..self.0.n_pendulum_digitization])
    }
    pub fn n_arm_vel(&self) -> usize {
        np::digitize(self.0.arm_vel.clamp(-8.0, 8.0), &np::linspace(-8.0, 8.0, self.0.n_arm_digitization + 1)[1..self.0.n_arm_digitization])
    }
    pub fn n_pendulum_vel(&self) -> usize {
        np::digitize(self.0.pendulum_vel.clamp(-8.0, 8.0), &np::linspace(-8.0, 8.0, self.0.n_pendulum_digitization + 1)[1..self.0.n_pendulum_digitization])
    }
    pub fn digitized_state(&self) -> usize {
        let d_arm = self.0.n_arm_digitization;
        let d_pendulum = self.0.n_pendulum_digitization;

        let n_arm_rad = self.n_arm_rad();
        let n_arm_vel = self.n_arm_vel();
        let n_pendulum_rad = self.n_pendulum_rad();
        let n_pendulum_vel = self.n_pendulum_vel();

        n_pendulum_rad + n_pendulum_vel * d_pendulum + n_arm_rad * d_pendulum.pow(2) + n_arm_vel * d_pendulum.pow(2) * d_arm
    }
    pub fn should_finish_episode(&self) -> bool {
        if self.n_arm_rad() <= 0 || self.0.n_arm_digitization <= self.n_arm_rad() {
            return true; // Arm is out of bounds
        }
        if self.n_pendulum_rad() <= 0 || self.0.n_pendulum_digitization <= self.n_pendulum_rad() {
            return true; // Pendulum is out of bounds
        }
        false
    }
}

pub struct AcrobotBalanceTask {
    pub n_arm_digitization: usize,
    pub n_pendulum_digitization: usize,
    pub action_size: usize,
    pub get_reward: fn(&AcrobotBalanceTask, state: &AcrobotState, action: &AcrobotAction) -> f64,
}
impl AcrobotBalanceTask {
    pub fn state(&self, observation: &AcrobotObservation) -> AcrobotState {
        AcrobotState::new(AcrobotStateInit {
            arm_rad: observation.shoulder_orientation.to_rad(),
            arm_vel: observation.shoulder_velocity,
            pendulum_rad: observation.elbow_orientation.to_rad(),
            pendulum_vel: observation.elbow_velocity,
            n_arm_digitization: self.n_arm_digitization,
            n_pendulum_digitization: self.n_pendulum_digitization,
        })
    }
}
impl oxide_control::Task for AcrobotBalanceTask {
    type Physics = Acrobot;
    type Observation = AcrobotObservation;
    type Action = AcrobotAction;

    fn discount(&self) -> f64 {
        0.99
    }

    fn init_episode(&self, physics: &mut Self::Physics) {
        let (elbow_id, shoulder_id) = (physics.elbow_id, physics.shoulder_id);
        physics.set_qpos(elbow_id, [np::random(-0.1, 0.1)]);
        physics.set_qvel(elbow_id, [np::random(-0.1, 0.1)]);
        physics.set_qpos(shoulder_id, [np::random(-0.1, 0.1)]);
        physics.set_qvel(shoulder_id, [np::random(-0.1, 0.1)]);
    }

    fn should_finish_episode(&self, observation: &Self::Observation) -> bool {
        self.state(observation).should_finish_episode()
    }

    fn get_reward(&self, observation: &Self::Observation, action: &Self::Action) -> f64 {
        let state = self.state(observation);
        (self.get_reward)(self, &state, action)
    }
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct AcrobotAction {
    #[serde(with = "serde_actuator_id")]
    actuator_id: ObjectId<obj::Actuator>,
    pub torque: f64,
    pub digitization_index: usize,
}
mod serde_actuator_id {
    use super::{ObjectId, obj};
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(actuator_id: &ObjectId<obj::Actuator>, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_u64(actuator_id.index() as u64)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<ObjectId<obj::Actuator>, D::Error> {
        let index = u64::deserialize(deserializer)?;
        Ok(unsafe { ObjectId::<obj::Actuator>::new_unchecked(index as usize) })
    }
}
impl oxide_control::Action for AcrobotAction {
    type Physics = Acrobot;

    fn apply(&self, actuators: &mut oxide_control::physics::Actuators<'_>) {
        assert!(!self.torque.is_nan(), "Torque cannot be NaN");
        assert!((-1.0..=1.0).contains(&self.torque), "Torque must be in the range [-1.0, 1.0]");
        actuators.set(self.actuator_id, self.torque);
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct QTableAgent {
    digitized_actions: Vec<AcrobotAction>,
    qtable: QTable,

    // for restored-from-file `TrainedAgent`...
    n_arm_digitization: usize,
    n_pendulum_digitization: usize,
}
pub struct QTableAgentConfig {
    pub action_size: usize,
    pub state_size: usize,
    pub initial_alpha: f64,
    pub initial_epsilon: f64,
}
impl QTableAgent {
    fn make_digitized_actions(acrobot: &Acrobot, config: &QTableAgentConfig) -> Vec<AcrobotAction> {
        let ctrlrange = acrobot.model().actuator_ctrlrange(acrobot.actuator_id).unwrap();
        assert_eq!(ctrlrange, -1.0..1.0);
        let digitized_torques = np::linspace(ctrlrange.start, ctrlrange.end, config.action_size);
        digitized_torques
            .into_iter()
            .enumerate()
            .map(|(index, torque)| AcrobotAction {
                actuator_id: acrobot.actuator_id,
                digitization_index: index,
                torque,
            })
            .collect::<Vec<_>>()
    }

    pub fn new(config: &QTableAgentConfig, env: &oxide_control::Environment<AcrobotBalanceTask>) -> Self {
        let qtable = QTable::new_with(QConfig {
            action_size: config.action_size,
            state_size: config.state_size,
            alpha: config.initial_alpha,
            epsilon: config.initial_epsilon,
            ..Default::default()
        });
        let digitized_actions = Self::make_digitized_actions(env.physics(), config);
        Self {
            qtable,
            digitized_actions,
            n_arm_digitization: env.task().n_arm_digitization,
            n_pendulum_digitization: env.task().n_pendulum_digitization,
        }
    }

    pub fn get_action<S: qtable::Strategy>(&self, state: AcrobotState) -> AcrobotAction {
        let qtable_state = qtable::State::new_on(&self.qtable, state.digitized_state()).unwrap();
        let qtable_action = self.qtable.next_action::<S>(qtable_state);
        self.digitized_actions[qtable_action.index()]
    }

    pub fn learn(&mut self, state: AcrobotState, action: AcrobotAction, reward: f64, next_state: AcrobotState) {
        self.qtable.update(QUpdate {
            state: qtable::State::new_on(&self.qtable, state.digitized_state()).unwrap(),
            action: qtable::Action::new_on(&self.qtable, action.digitization_index).unwrap(),
            reward,
            next_state: qtable::State::new_on(&self.qtable, next_state.digitized_state()).unwrap(),
        });
    }
    pub fn decay_alpha_with_rate(&mut self, rate: f64) {
        self.qtable.decay_alpha_with_rate(rate);
    }
    pub fn decay_epsilon_with_rate(&mut self, rate: f64) {
        self.qtable.decay_epsilon_with_rate(rate);
    }

    pub fn save(&self, file_path: impl AsRef<std::path::Path>) -> Result<(), std::io::Error> {
        serde_json::to_writer(std::fs::File::create(file_path)?, self).map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
        Ok(())
    }
    pub fn load(qtable_filename: impl AsRef<std::path::Path>) -> Result<Self, std::io::Error> {
        serde_json::from_reader::<_, QTableAgent>(std::fs::File::open(qtable_filename)?).map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))
    }
}

pub struct TrainedAgent(QTableAgent);
impl TrainedAgent {
    pub fn new(qtable_agent: QTableAgent) -> Self {
        Self(qtable_agent)
    }

    pub fn load(qtable_filename: impl AsRef<std::path::Path>) -> Result<Self, std::io::Error> {
        QTableAgent::load(qtable_filename).map(Self)
    }

    pub fn action_size(&self) -> usize {
        self.0.digitized_actions.len()
    }
    pub fn n_arm_digitization(&self) -> usize {
        self.0.n_arm_digitization
    }
    pub fn n_pendulum_digitization(&self) -> usize {
        self.0.n_pendulum_digitization
    }

    pub fn get_action(&self, state: AcrobotState) -> AcrobotAction {
        self.0.get_action::<qtable::strategy::MostQValue>(state)
    }
}
