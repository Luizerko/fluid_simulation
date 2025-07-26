use eframe::{egui, App, CreationContext, Frame, NativeOptions};
use egui::{Pos2, Vec2, Color32};
use std::time::Duration;
use rand::{Rng, thread_rng};

// Datas tructure for particles 
struct Particle {
    position: Pos2,
    velocity: Vec2,
    radius: f32,
    weight: f32,
    color: Color32
}

// Physics trait
trait Physics {
    fn update(&mut self, dt: f32);
}

// Implementing physics dynamics for the particles
impl Physics for Particle {
    fn update(&mut self, dt: f32) {
        self.position += self.velocity*dt;
    }
}

// Datastructure for all particles in the fluid simulation
struct FluidSim {
    particles: Vec<Particle>
}

// Implementing particle generation
impl FluidSim {
    fn generate_particles(count: usize) -> Vec<Particle> {
        let mut particles = Vec::with_capacity(count);
        let mut rng = thread_rng();
        
        for c in 0..count {
            let p = Particle {
                position: Pos2::new(100.0 + c as f32 * 4.0, 100.0 + c as f32 * 4.0),
                velocity: Vec2::new(10.0, 10.0),
                radius: 5.0,
                weight: 1.0,
                color: Color32::from_rgb(100, 100, 100)
            };
            particles.push(p);
        }

        particles
    }
}

// Implementing window event loop
impl App for FluidSim {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
        // Getting timestep
        let dt = ctx.input(|i| i.stable_dt);
        
        // Drawing particles
        egui::CentralPanel::default().show(ctx, |ui| {
            let painter = ui.painter();
            for p in &self.particles {
                painter.circle_filled(p.position, p.radius, p.color);
            }
        });

        // Updating positions
        for p in &mut self.particles {
            p.update(dt);
        }

        // Breaking the reactive mode and running simulation at 30 FPS
        ctx.request_repaint_after(Duration::from_secs_f32(1.0/30.0));
    }
}

// Initializing the system
fn main() -> eframe::Result<()> {
    let options = NativeOptions::default();
    eframe::run_native(
        "Fluid Simulation",
        options,
        Box::new(|_cc: &CreationContext<'_>| {
            let particle_count = 10;
            let sim = FluidSim {
                particles: FluidSim::generate_particles(particle_count),
            };
            Ok(Box::new(sim))
        }),
    )
}