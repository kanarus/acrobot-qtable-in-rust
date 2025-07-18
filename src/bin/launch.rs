use acrobot_qtable::TrainedAgent;
use quarc::{Q2Usb, QErr};
use std::f64::consts::PI;

struct InvertedPendulum<C: InvertedPendulumController> {
    iodevice: Q2Usb,
    controller: C,
    torque: f64,
    state: InvertedPendulumState,
    log: InvertedPendulumLog,
}
trait InvertedPendulumController {
    fn determine_torque(&self, state: &InvertedPendulumState) -> f64;
    fn after_termination(&self, state: &InvertedPendulumState, log: &InvertedPendulumLog);
}
struct InvertedPendulumState {
    theta: f64,
    alpha: f64,
    thetadot: f64,
    alphadot: f64,
}
struct InvertedPendulumLog {
    theta: f64,
    alpha: f64,
    thetadot: f64,
    alphadot: f64,
    alphaf: f64,
    torque: f64,
    voltage: f64,
    time: f64,
    dt: f64,
}

impl<C: InvertedPendulumController> InvertedPendulum<C> {
    /* mechanical parameters of the inverted pendulum */
    const LR: f64 = 0.2159;
    const JR: f64 = 9.9829e-4;
    const BR: f64 = 2.4e-3;
    const MR: f64 = 0.2570;
    const LP: f64 = 0.3365;
    const JP: f64 = 1.2e-3;
    const BP: f64 = 2.4e-3;
    const MP: f64 = 0.1270;

    /* parameters related to conversion from voltage to torque */
    const ETA_G: f64 = 0.69;
    const ETA_M: f64 = 0.9;
    const RM: f64 = 2.6;
    const KG: f64 = 70.0;
    const KT: f64 = 7.68e-3;
    const KM: f64 = 7.68e-3;
    const AM: f64 = Self::KG * Self::KT / Self::RM;
    const BM: f64 = Self::KG * Self::KM;

    /* acceleration of gravity */
    const G: f64 = 9.81;

    fn new(controller: C) -> Result<Self, QErr> {
        let iodevice = Q2Usb::new()?;
        Ok(Self {
            iodevice,
            controller,
            torque: 0.,
            state: InvertedPendulumState::new(),
            log: InvertedPendulumLog {
                theta: 0.,
                alpha: PI,
                thetadot: 0.,
                alphadot: 0.,
                alphaf: 0.,
                torque: 0.,
                voltage: 0.,
                time: 0.,
                dt: 0.,
            },
        })
    }

    fn torque_to_voltage(&self) -> f64 {
        self.torque / Self::AM + self.state.thetadot * Self::BM
    }
}
impl InvertedPendulumState {
    fn new() -> Self {
        Self {
            theta: 0.,
            alpha: 0.,
            thetadot: 0.,
            alphadot: 0.,
        }
    }
    fn from_theta_alphaf(&self, theta: f64, alphaf: f64) -> Self {
        let alpha = alphaf % (2. * PI) - PI;
        let thetadot = s
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let target_agent_path = args.next().expect("Usage: simulate <target_agent_path>");

    println!("Loading trained agent from `{target_agent_path}`...");

    let t = TrainedAgent::load(&target_agent_path).expect(&format!("Failed to load trained agent from file `{target_agent_path}`"));

    let mut q2usb = Q2Usb::new().expect("can't initialize Q2Usb");

    todo!()
}
