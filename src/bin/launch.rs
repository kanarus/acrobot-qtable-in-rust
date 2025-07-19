use acrobot_qtable::{AcrobotState, AcrobotStateInit, TrainedAgent};
use quarc::{Q2Usb, QErr};
use std::f64::consts::PI;

struct InvertedPendulum {
    iodevice: Q2Usb,
    torque: f64,
    state: InvertedPendulumState,
    log: InvertedPendulumLog,
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

impl InvertedPendulum {
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
}

impl InvertedPendulum {
    fn new() -> Result<Self, QErr> {
        let iodevice = Q2Usb::new()?;
        Ok(Self {
            iodevice,
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
}
impl InvertedPendulumLog {
    fn new() -> Self {
        Self {
            theta: 0.,
            alpha: PI,
            thetadot: 0.,
            alphadot: 0.,
            alphaf: 0.,
            torque: 0.,
            voltage: 0.,
            time: 0.,
            dt: 0.,
        }
    }
}

trait InvertedPendulumController {
    fn determine_torque(&self, state: &InvertedPendulumState) -> f64;
    #[allow(unused_variables)]
    fn after_termination(&self, state: &InvertedPendulumState, log: &InvertedPendulumLog) {}
}

struct RunConfig {
    sample_time: f64,
    simulation_time: f64,
}
impl Default for RunConfig {
    fn default() -> Self {
        Self {
            sample_time: 0.05,
            simulation_time: 1.0,
        }
    }
}

impl InvertedPendulum {
    /// convert current `torque` to voltage, and input it to the device
    fn input_torque(&mut self) -> Result<(), QErr> {
        let voltage = self.torque / Self::AM + self.state.thetadot * Self::BM;
        self.iodevice.write_voltage(voltage.clamp(-5.0, 5.0))?;
        Ok(())
    }

    fn run(self, controller: impl InvertedPendulumController, config: RunConfig) -> Result<(), QErr> {
        struct Run<C: InvertedPendulumController> {
            inverted_pendulum: InvertedPendulum,
            controller: C,
            config: RunConfig,
        }
        impl<C: InvertedPendulumController> Drop for Run<C> {
            // Automatically called when the simulation ends, either normally or due to an error / early termination.
            fn drop(&mut self) {
                self.controller.after_termination(&self.inverted_pendulum.state, &self.inverted_pendulum.log);
            }
        }

        let mut prev_theta = self.log.theta;
        let mut prev_alpha = self.log.alpha;
        let mut prev_elapsed_secs = 0.0;

        let mut r = Run {
            inverted_pendulum: self,
            controller,
            config,
        };

        let t = std::time::Instant::now();
        while prev_elapsed_secs < r.config.simulation_time {
            let dt = {
                let elapsed_secs = t.elapsed().as_secs_f64();
                let dt = elapsed_secs - prev_elapsed_secs;
                prev_elapsed_secs = elapsed_secs;
                dt
            };

            let (alpha, theta, alphadot, thetadot) = {
                let (theta, alphaf) = r.inverted_pendulum.iodevice.read_theta_and_alpha()?;
                let alpha = alphaf % (2.0 * PI) - PI;
                let thetadot = (theta - prev_theta) / dt;
                let alphadot = (alpha - prev_alpha) / dt;
                prev_theta = theta;
                prev_alpha = alpha;
                (alpha, theta, alphadot, thetadot)
            };

            if theta.abs() > PI {
                println!("Emergency stop: theta out of bounds (|theta| > PI)");
                break;
            }
        }

        Ok(())
    }
}

impl InvertedPendulumController for TrainedAgent {
    fn determine_torque(&self, state: &InvertedPendulumState) -> f64 {
        let state = AcrobotState::new(AcrobotStateInit {
            arm_rad: state.theta,
            arm_vel: state.thetadot,
            pendulum_rad: state.alpha,
            pendulum_vel: state.alphadot,
            n_arm_digitization: self.n_arm_digitization(),
            n_pendulum_digitization: self.n_pendulum_digitization(),
        });
        if state.should_finish_episode() { 0.0 } else { self.get_action(state).torque }
    }
}

fn main() -> Result<(), QErr> {
    let mut args = std::env::args().skip(1);
    let target_agent_path = args.next().expect("Usage: simulate <target_agent_path>");

    println!("Loading trained agent from `{target_agent_path}`...");
    let controller = TrainedAgent::load(&target_agent_path).expect(&format!("Failed to load trained agent from file `{target_agent_path}`"));

    println!("Connecting to the interted pendulum...");
    let ip = InvertedPendulum::new()?;

    ip.run(controller, Default::default())?;

    println!("Simulation completed successfully.");
    Ok(())
}
