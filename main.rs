use eframe::{egui, App, CreationContext, Frame, NativeOptions};
use egui::{Pos2, Vec2, Color32, ViewportBuilder};
use std::time::Duration;
use rand::{Rng, rng};

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
    fn update(&mut self, dt: f32, e: f32, bounds: egui::Rect);
}

// Implementing physics dynamics for the particles
impl Physics for Particle {
    fn update(&mut self, dt: f32, e: f32, bounds: egui::Rect) {
        // Applying gravity
        let g = 9.81;
        self.velocity.y += self.mass * g * dt;

        // Updating positions
        self.position += self.velocity*dt;

        // Treating wall collisions (elastic for now)
        if self.position.x < 0.0 {
            self.velocity.x = -e*self.velocity.x;
            self.position.x = self.radius;
        }
        else if self.position.x > bounds.width() {
            self.velocity.x = -e*self.velocity.x;
            self.position.x = bounds.width() - self.radius;
        }
        
        if self.position.y < 0.0 {
            self.velocity.y = -e*self.velocity.y;
            self.position.y = self.radius;
        }
        else if self.position.y > bounds.height() {
            self.velocity.y = -e*self.velocity.y;
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
    fn generate_particles(count: usize, width: f32, height: f32) -> Vec<Particle> {
        let mut particles = Vec::with_capacity(count);
        let mut local_rng = rng();

        for _c in 0..count {
            let p = Particle {
                position: Pos2::new(local_rng.random_range(0.0..width), local_rng.random_range(0.0..height)),
                velocity: Vec2::new(0.0, 0.0),
                radius: 2.0,
                mass: 10.0,
                color: Color32::from_rgb(255, 0, 0)
            };
            particles.push(p);
        }

        particles
    }

    // Applying interactions with other particles
    fn handle_collisions(&mut self, e: f32) {
        // Loop over all particles (we'll make this better, of course)
        for i in 0..self.particles.len() {
            for j in (i+1)..self.particles.len() {
                let (head, tail) = self.particles.split_at_mut(j);
                let p1: &mut Particle = &mut head[i];
                let p2: &mut Particle = &mut tail[0];
                
                // Compute collisions based on distance between centers and radius
                let delta = p1.position - p2.position;
                let dist_sq = delta.x*delta.x + delta.y*delta.y;
                let radii_sum = p1.radius + p2.radius;  
                
                // Check for collision
                if dist_sq < radii_sum * radii_sum {
                    // Get the distance (with no division by zero) and compute the collision vector
                    let dist = dist_sq.sqrt().max(1e-6);
                    let n = delta/dist;

                    // Finding relative velocity, projecting it along collision vector and computing impulse magnitude (considering conservation of momentum)
                    let v_rel = p1.velocity - p2.velocity;
                    let v_rel_proj_n = v_rel.dot(n);
                    
                    let inv_m1 = 1.0/p1.mass;
                    let inv_m2 = 1.0/p2.mass;
                    let j = -(1.0+e) * v_rel_proj_n / (inv_m1+inv_m2);

                    // Updating velocities based on collision vector, impulse and particles' masses
                    p1.velocity += n*inv_m1*j;
                    p2.velocity += -n*inv_m2*j;

                    // Position correction to avoid overlapping particles
                    let penetration = (radii_sum - dist).max(0.0);
                    let correction = n * (penetration/(inv_m1 + inv_m2));
                    p1.position += correction * inv_m1;
                    p2.position -= correction * inv_m2;
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
        
        // Defining collision elasticity
        let e = 0.4;

        // Updating positions
        for p in &mut self.particles {
            p.update(dt, e, window_size);
        }

        // Handling collisions with other particles
        self.handle_collisions(e);

        // Breaking the reactive mode and running simulation at 60 FPS
        ctx.request_repaint_after(Duration::from_secs_f32(1.0/60.0));
    }
}

// Initializing the system
fn main() -> eframe::Result<()> {
    let mut options = NativeOptions::default();
    let width = 1920.0;
    let height = 1080.0;
    options.viewport = ViewportBuilder::default().with_inner_size(Vec2::new(width, height));
    
    eframe::run_native(
        "Fluid Simulation",
        options,
        Box::new(|_cc: &CreationContext<'_>| {
            let particle_count = 1000;
            let sim = FluidSim {
                particles: FluidSim::generate_particles(particle_count, width, height),
            };
            Ok(Box::new(sim))
        }),
    )
}