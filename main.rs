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
    fn update_position(&mut self, dt: f32, e: f32, bounds: egui::Rect);
    fn update_properties(&mut self, radius: f32, mass: f32, color: Color32);
}

// Implementing physics dynamics for the particles
impl Physics for Particle {
    fn update_position(&mut self, dt: f32, e: f32, bounds: egui::Rect) {
        // Applying gravity
        let g = 9.81;
        self.velocity.y += self.mass * g * dt;

        // Updating positions
        self.position += self.velocity*dt;

        // Overlapping with the wall check variables
        let left_overlap = self.position.x - self.radius;
        let right_overlap = self.position.x + self.radius - bounds.width();
        let top_overlap = self.position.y - self.radius;
        let bot_overlap = self.position.y + self.radius - bounds.height();

        // Treating wall collisions (elastic for now)
        if left_overlap < 0.0 {
            self.velocity.x = -e*self.velocity.x;
            self.position.x += left_overlap.abs();
        }
        else if right_overlap > 0.0 {
            self.velocity.x = -e*self.velocity.x;
            self.position.x -= right_overlap;
        }
        
        if top_overlap < 0.0 {
            self.velocity.y = -e*self.velocity.y;
            self.position.y += top_overlap.abs();
        }
        else if bot_overlap > 0.0 {
            self.velocity.y = -e*self.velocity.y;
            self.position.y -= bot_overlap;
        }
    }

    // Updating properties of particles
    fn update_properties(&mut self, radius: f32, mass: f32, color: Color32) {
        self.radius = radius;
        self.mass = mass;
        self.color = color;
    }
}

// Datastructure for all particles in the fluid simulation
struct FluidSim {
    particles: Vec<Particle>,
    default_radius: f32,
    default_mass: f32,
    default_color: Color32,
    r: u8,
    g: u8,
    b: u8,
    restitution: f32
}

impl FluidSim {
    // Instantiating empty structure
    fn new_empty(radius: f32, mass: f32, color: Color32, restitution: f32) -> Self {
        Self {
            particles: Vec::new(),
            default_radius: radius,
            default_mass: mass,
            default_color: color,
            r: color.r(),
            g: color.g(),
            b: color.b(),
            restitution,
        }
    }

    // Implementing particle generation
    fn generate_particles(&self, count: usize, width: f32, height: f32) -> Vec<Particle> {
        let mut particles = Vec::with_capacity(count);
        let mut local_rng = rng();

        for _c in 0..count {
            let p = Particle {
                position: Pos2::new(local_rng.random_range(0.0..width), local_rng.random_range(0.0..height)),
                velocity: Vec2::new(0.0, 0.0),
                radius: self.default_radius,
                mass: self.default_mass,
                color: self.default_color
            };
            particles.push(p);
        }

        particles
    }

    // Reseting simulation
    fn reset_sim(&mut self, count: usize, width: f32, height: f32) {
        self.particles = self.generate_particles(count, width, height);
    }

    // Applying interactions with other particles
    fn handle_collisions(&mut self) {
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
                    let j = -(1.0+self.restitution) * v_rel_proj_n / (inv_m1+inv_m2);

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

        // Getting window information
        let window_size: egui::Rect = ctx.screen_rect();
        
        // Spawining points while clicking
        let pointer = ctx.pointer_interact_pos();
        if ctx.input(|i| i.pointer.primary_pressed()) {
            if let Some(pos) = pointer {
                    self.particles.push(Particle {
                        position: Pos2::new(pos.x, pos.y),
                        velocity: Vec2::new(0.0, 0.0),
                        radius: self.default_radius,
                        mass: self.default_mass,
                        color: self.default_color,
                    });
            }
        }

        // Creating interactive interface
        egui::CentralPanel::default().show(ctx, |ui| {
            // General infos and buttons
            ui.horizontal(|ui| {
                ui.label(format!("Number of Particles: {}", self.particles.len()));
                ui.add_space(20.0);
                if ui.button("Reset Simulation").clicked() {
                    self.reset_sim(self.particles.len(), window_size.width(), window_size.height());
                }
            });

            // Particle radius slider and mass slider
            ui.horizontal(|ui| {
                ui.label("Radius:");
                ui.add(egui::Slider::new(&mut self.default_radius, 1.0..=5.0));
                ui.add_space(10.0);
                ui.label("Mass:");
                ui.add(egui::Slider::new(&mut self.default_mass, 10.0..=50.0));
            });

            // Particle color sliders
            ui.horizontal(|ui| {
                ui.label("R:");
                ui.add(egui::Slider::new(&mut self.r, 0..=255));
                ui.add_space(10.0);

                ui.label("G:");
                ui.add(egui::Slider::new(&mut self.g, 0..=255));
                ui.add_space(10.0);

                ui.label("B:");
                ui.add(egui::Slider::new(&mut self.b, 0..=255));

                self.default_color = Color32::from_rgb(self.r, self.g, self.b);
            });

            // Collision coefficient slider
            ui.horizontal(|ui| {
                ui.label("Restitution Coefficient (e):");
                ui.add(egui::Slider::new(&mut self.restitution, 0.0..=1.0));
            });

            // Drawing particles
            let painter = ui.painter();
            for p in &self.particles {
                painter.circle_filled(p.position, p.radius, p.color);
            }
        });

        // Deciding whether or not to update properties of particles
        if self.particles.len() > 0 && (self.default_radius != self.particles[0].radius || self.default_mass != self.particles[0].mass || self.default_color != self.particles[0].color) {
            // Updating particle dynamics and properties on a timestep
            for p in &mut self.particles {
                // Updating properties
                p.update_properties(self.default_radius, self.default_mass, self.default_color);
                
                // Updating positions
                p.update_position(dt, self.restitution, window_size);
            }
        }
        else {
            // Updating particle dynamics
            for p in &mut self.particles {
                // Updating positions
                p.update_position(dt, self.restitution, window_size);
            }
        }


        // Handling collisions with other particles
        self.handle_collisions();

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
            let particle_count = 0;
            let default_radius = 2.0;
            let default_mass = 10.0;
            let default_color = Color32::from_rgb(35, 137, 218);
            let restitution = 0.4;
            let mut sim = FluidSim::new_empty(default_radius, default_mass, default_color, restitution);
            sim.reset_sim(particle_count, width, height);
            Ok(Box::new(sim))
        }),
    )
}