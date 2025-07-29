use eframe::{egui, App, CreationContext, Frame, NativeOptions};
use egui::{Pos2, Vec2, Color32};
use std::time::Duration;
use rand::{Rng, thread_rng};

// Datas tructure for particles 
struct Particle {
    position: Pos2,
    velocity: Vec2,
    radius: f32,
    mass: f32,
    color: Color32
}

// Physics trait
trait Physics {
    fn update(&mut self, dt: f32, bounds: egui::Rect);
}

// Implementing physics dynamics for the particles
impl Physics for Particle {
    fn update(&mut self, dt: f32, bounds: egui::Rect) {
        // Applying gravity
        let g = 9.81;
        self.velocity.y += self.mass * g * dt;

        // Updating positions
        self.position += self.velocity*dt;

        // Treating wall collisions (elastic for now)
        if self.position.x < 0.0 {
            self.velocity.x = -self.velocity.x;
            self.position.x = self.radius;
        }
        else if self.position.x > bounds.width() {
            self.velocity.x = -self.velocity.x;
            self.position.x = bounds.width() - self.radius;
        }
        
        if self.position.y < 0.0 {
            self.velocity.y = -self.velocity.y;
            self.position.y = self.radius;
        }
        else if self.position.y > bounds.height() {
            self.velocity.y = -self.velocity.y;
            self.position.y = bounds.height() - self.radius;
        }
    }
}

// Datastructure for all particles in the fluid simulation
struct FluidSim {
    particles: Vec<Particle>
}

impl FluidSim {
    // Implementing particle generation
    fn generate_particles(count: usize) -> Vec<Particle> {
        let mut particles = Vec::with_capacity(count);
        let mut rng = thread_rng();
        
        for c in 0..count {
            let p = Particle {
                position: Pos2::new(100.0 + c as f32 * 10.0, 100.0 + c as f32 * 10.0),
                velocity: Vec2::new(100.0, 100.0),
                radius: 5.0,
                mass: 10.0,
                color: Color32::from_rgb(100, 100, 100)
            };
            particles.push(p);
        }

        particles
    }

    // Applying interactions with other particles
    fn handle_collisions(&mut self) {
        // Loop over all particles (we'll make this better, of course)
        for i in 0..self.particles.len() {
            for j in (i+1)..self.particles.len() {
                let (p1, p2) = (&mut self.particles[i], &mut self.particles[j]);
                
                // Compute collisions based on distance between centers and radius
                let delta = p1.position - p2.position;
                let dist_sq = delta.x*delta.x + delta.y*delta.y;
                let radii_sum = p1.radius + p2.radius;  
                
                // Check for collision
                if dist_sq < radii_sum * radii_sum {
                    // Get the distance (with no division by zero) and compute the unit vector (direction) between centers
                    let dist = dist_sq.sqrt().max(0.0001);
                    let n = delta / dist;

                    
                }
            }
        }
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

        // Getting window information to check corner cases on position updates
        let window_size: egui::Rect = ctx.screen_rect();
        
        // Updating positions
        for p in &mut self.particles {
            p.update(dt, window_size);
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