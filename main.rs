use eframe::{egui, App, CreationContext, Frame, NativeOptions};
use egui::{Pos2, Vec2, Color32, ViewportBuilder};

use std::time::{Duration, Instant};
use std::collections::HashMap;

use rand::{Rng, rng};

// Datas tructure for particles 
struct Particle {
    position: Pos2,
    prev_position: Pos2,
    velocity: Vec2,
    radius: f32,
    mass: f32,
    color: Color32
}

// Physics trait
trait Physics {
    fn update_position(&mut self, dt: f32, g:f32, e: f32, bounds: egui::Rect);
    fn update_properties(&mut self, radius: f32, color: Color32);
}

// Implementing physics dynamics for the particles using position-Verlet
impl Physics for Particle {
    fn update_position(&mut self, dt: f32, g: f32, e: f32, bounds: egui::Rect) {
        // Applying gravity in pixels/s²
        let a = Vec2::new(0.0, g);
        
        // Update equation: x_{n+1} = 2 x_n - x_{n-1} + a dt^2
        let x_prev = self.prev_position;
        let x_curr = self.position;
        let x_next = x_curr + (x_curr - x_prev) + a * (dt * dt);
        self.prev_position = x_curr;
        self.position = x_next;

        // Update velocity equation at time t_n (O(dt^2)): v_n ≈ (x_{n+1} - x_{n-1}) / (2 dt)
        self.velocity = (self.position - x_prev) * (0.5 / dt);

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

        // Keeping Verlet state (previous step) consistent with wall collision with (position - prev_position)/dt == velocity
        self.prev_position = self.position - self.velocity * dt;
    }

    // Updating properties of particles
    fn update_properties(&mut self, radius: f32, color: Color32) {
        self.radius = radius;
        self.color = color;
    }
}

// Data structure for spwan mode
#[derive(PartialEq, Eq)]
enum SpawnMode {
    Point,
    Region,
    Flow
}

// Data structure for all particles in the fluid simulation
struct FluidSim {
    default_phys_dt: f32,
    default_g: f32,
    particles: Vec<Particle>,
    default_radius: f32,
    default_mass: f32,
    default_color: Color32,
    r: u8,
    g: u8,
    b: u8,
    restitution: f32,
    spawn_mode: SpawnMode,
    drag_start: Pos2,
    drag_current: Pos2,
    last_frame: Instant,
    fps: f32,
    fps_accum_time: f32,
    fps_accum_frames: u32,
    accumulator: f32,
    max_substeps: u32,
    max_frame_dt: f32
}

impl FluidSim {
    // Instantiating empty structure
    fn new_empty(phys_dt: f32, g: f32, radius: f32, mass: f32, color: Color32, restitution: f32) -> Self {
        Self {
            default_phys_dt: phys_dt,
            default_g: g,
            particles: Vec::new(),
            default_radius: radius,
            default_mass: mass,
            default_color: color,
            r: color.r(),
            g: color.g(),
            b: color.b(),
            restitution: restitution,
            spawn_mode: SpawnMode::Point,
            drag_start: Pos2::new(0.0, 0.0),
            drag_current: Pos2::new(0.0, 0.0),
            last_frame: Instant::now(),
            fps: 0.0,
            fps_accum_time: 0.0,
            fps_accum_frames: 0,
            accumulator: 0.0,
            max_substeps: 32,
            max_frame_dt: 0.1
        }
    }

    // Helper function to bootstrap prev_position for a new particle (x_{-1})
    fn bootstrap_prev_position(pos: Pos2, vel: Vec2, dt: f32, g: f32) -> Pos2 {
        // x(t - dt) = x(t) - v(t) dt + 1/2 a dt^2
        let a = Vec2::new(0.0, g);
        pos - vel * dt + a * (0.5 * dt * dt)
    }

    // Reseting random simulation
    fn reset_random_sim(&mut self, count: usize, width: f32, height: f32) {
        self.particles = Vec::with_capacity(count);
        let mut local_rng = rng();

        for _c in 0..count {
            let position = Pos2::new(local_rng.random_range(0.0..width), local_rng.random_range(0.0..height));
            let velocity = Vec2::new(0.0, 0.0);
            let prev_position = Self::bootstrap_prev_position(position, velocity, self.default_phys_dt, self.default_g);
            let p = Particle {
                position,
                prev_position,
                velocity,
                radius: self.default_radius,
                mass: self.default_mass,
                color: self.default_color
            };
            self.particles.push(p);
        }
    }

    // Reseting empty simulation
    fn reset_empty_sim(&mut self) {
        self.particles = Vec::new();
    }

    // Applying interactions with other particles
    fn handle_collisions(&mut self, dt: f32) {
        // Defining grid cell size 
        let cell_size = 2.0 * self.default_radius;

        // Building spatial hash grid: (cell_x, cell_y) -> list of particle indices
        let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();

        for idx in 0..self.particles.len() {
            let p = &self.particles[idx];
            let cell_x = (p.position.x/cell_size).floor() as i32;
            let cell_y = (p.position.y/cell_size).floor() as i32;
            grid.entry((cell_x, cell_y)).or_insert_with(Vec::new).push(idx);
        }

        // For each particle, testing only against neighbors in 3x3 neighborhood
        for idx in 0..self.particles.len() {
            let p1 = &self.particles[idx];
            let cell_x = (p1.position.x/cell_size).floor() as i32;
            let cell_y = (p1.position.y/cell_size).floor() as i32;

            for dx in -1..=1 {
                for dy in -1..=1 {
                    let neighbor_cell = (cell_x + dx, cell_y + dy);
                    if let Some(candidates) = grid.get(&neighbor_cell) {
                        for &j in candidates {
                            // Avoiding updates we already made 
                            if j <= idx {
                                continue;
                            }
                            
                            let (head, tail) = self.particles.split_at_mut(j);
                            let p1_mut: &mut Particle = &mut head[idx];
                            let p2_mut: &mut Particle = &mut tail[0];

                            // Compute collisions based on distance between centers and radius
                            let delta = p1_mut.position - p2_mut.position;
                            let dist_sq = delta.x*delta.x + delta.y*delta.y;
                            let radii_sum = p1_mut.radius + p2_mut.radius;  
                            
                            // Check for collision
                            if dist_sq < radii_sum * radii_sum {
                                // Get the distance (with no division by zero) and compute the collision vector
                                let dist = dist_sq.sqrt().max(1e-6);
                                let n = delta/dist;

                                // Finding relative velocity, projecting it along collision vector and computing inverse mass
                                let v_rel = p1_mut.velocity - p2_mut.velocity;
                                let v_rel_proj_n = v_rel.dot(n);
                                
                                let inv_m1 = 1.0/p1_mut.mass;
                                let inv_m2 = 1.0/p2_mut.mass;

                                // Position correction to avoid overlapping particles
                                let penetration = (radii_sum - dist).max(0.0);
                                let correction = n * (penetration/(inv_m1 + inv_m2));
                                p1_mut.position += correction * inv_m1;
                                p2_mut.position -= correction * inv_m2;

                                // If particles were already moving away, they just collided because of local interactions with other particles
                                if v_rel_proj_n > 0.0 {
                                    continue;
                                }

                                // Computing impulse magnitude (considering conservation of momentum) and updating velocities based on collision vector, impulse and particles' masses
                                let j = -(1.0+self.restitution) * v_rel_proj_n / (inv_m1+inv_m2);
                                p1_mut.velocity += n*inv_m1*j;
                                p2_mut.velocity += -n*inv_m2*j;
                            
                                // Keeping Verlet state (previous step) consistent with new v again
                                p1_mut.prev_position = p1_mut.position - p1_mut.velocity * dt;
                                p2_mut.prev_position = p2_mut.position - p2_mut.velocity * dt;
                            }
                        }
                    }
                }
            }
        }
    }

    // Implementing physics simulation step to separate physics update from rendering update
    fn physics_step(&mut self, dt: f32, bounds: egui::Rect) {
        // Updating positions
        for p in &mut self.particles {
            p.update_position(dt, self.default_g, self.restitution, bounds);
        }
        // Handling collisions with other particles
        self.handle_collisions(dt);
    }
}

// Implementing window event loop
impl App for FluidSim {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
        // Computing real time between frames and clamping spikes so we don't try to catch up forever
        let now = Instant::now();
        let real_dt = (now - self.last_frame).as_secs_f32().min(self.max_frame_dt);
        self.accumulator += real_dt;
        self.last_frame = now;

        // Accumulating into a short window for a stable read
        self.fps_accum_time += real_dt;
        self.fps_accum_frames += 1;
        if self.fps_accum_time >= 0.5 {
            self.fps = self.fps_accum_frames as f32 / self.fps_accum_time;
            self.fps_accum_time = 0.0;
            self.fps_accum_frames = 0;
        }

        // Getting window information
        let window_size: egui::Rect = ctx.screen_rect();

        // Spawning points
        let pointer = ctx.pointer_interact_pos();
        let primary_pressed = ctx.input(|i| i.pointer.primary_pressed());
        let primary_down = ctx.input(|i| i.pointer.primary_down());
        let primary_released = ctx.input(|i| i.pointer.primary_released());

        // Avoiding spawning points on menu
        if let Some(pos) = pointer {
            if pos.x > 580.0 || pos.y > 150.0 {
                // Individually when clicking
                if self.spawn_mode == SpawnMode::Point {
                    if primary_pressed {
                        let position = Pos2::new(pos.x, pos.y);
                        let velocity = Vec2::new(0.0, 0.0);
                        let prev_position = FluidSim::bootstrap_prev_position(position, velocity, self.default_phys_dt, self.default_g);
                        self.particles.push(Particle {
                            position,
                            prev_position,
                            velocity,
                            radius: self.default_radius,
                            mass: self.default_mass,
                            color: self.default_color,
                        });
                    }
                }

                // By region selection
                else if self.spawn_mode == SpawnMode::Region {
                    // Starting mouse capture
                    if primary_pressed {
                        self.drag_start = pos;
                        self.drag_current = pos;
                    }

                    // Updating mouse position
                    if primary_down {
                        self.drag_current = pos;
                    }

                    // Generting points after mouse release
                    if primary_released {
                        // Computing bounds (considering multiple drag directions)
                        let (start, end) = (self.drag_start, self.drag_current);
                        let min_x = start.x.min(end.x);
                        let max_x = start.x.max(end.x);
                        let min_y = start.y.min(end.y);
                        let max_y = start.y.max(end.y);

                        // Choosing a spacing so not to create billions of particles and adding random jitter so not to get a completly uniform particle generation
                        let mut local_rng = rng();
                        let step = self.default_radius * 2.0;
                        let mut y = min_y;
                        while y <= max_y {
                            let mut x = min_x;
                            while x <= max_x {
                                let jitter = Vec2::new(local_rng.random_range(-2.0..=2.0), local_rng.random_range(-2.0..=2.0));
                                let position = Pos2::new(x, y) + jitter;
                                let velocity = Vec2::new(0.0, 0.0);
                                let prev_position = FluidSim::bootstrap_prev_position(position, velocity, self.default_phys_dt, self.default_g);
                                self.particles.push(Particle {
                                    position,
                                    prev_position,
                                    velocity,
                                    radius: self.default_radius,
                                    mass: self.default_mass,
                                    color: self.default_color,
                                });
                                x += step;
                            }
                            y += step;
                        }
                        
                        // Clearing drag state
                        self.drag_start = Pos2::new(0.0, 0.0);
                        self.drag_current = Pos2::new(0.0, 0.0);
                    }
                }
            }
        }

        // Creating interactive interface
        egui::CentralPanel::default().show(ctx, |ui| {
            // General infos and buttons
            ui.horizontal(|ui| {
                ui.label(format!("FPS: {:>5.1}", self.fps));
                ui.add_space(20.0);
                ui.label(format!("Number of Particles: {}", self.particles.len()));
                ui.add_space(20.0);
                if ui.button("Reset Random Simulation").clicked() {
                    self.reset_random_sim(self.particles.len(), window_size.width(), window_size.height());
                }
                ui.add_space(20.0);
                if ui.button("Reset Empty Simulation").clicked() {
                    self.reset_empty_sim();
                }
            });

            // Particle radius slider and mass slider
            ui.horizontal(|ui| {
                ui.label("Radius:");
                ui.add(egui::Slider::new(&mut self.default_radius, 1.0..=5.0));
                ui.add_space(10.0);
                ui.label("Gravity (pixels/s²):");
                ui.add(egui::Slider::new(&mut self.default_g, 400.0..=1200.0));
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

            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.spawn_mode, SpawnMode::Point, "Point");
                ui.add_space(10.0);
                ui.selectable_value(&mut self.spawn_mode, SpawnMode::Region, "Region");
                ui.add_space(10.0);
                ui.selectable_value(&mut self.spawn_mode, SpawnMode::Flow, "Flow");
            });

            // Drawing particles
            let painter = ui.painter();
            let mut shapes = Vec::with_capacity(self.particles.len());
            for p in &self.particles {
                //painter.circle_filled(p.position, p.radius, p.color);
                shapes.push(egui::Shape::circle_filled(p.position, p.radius, p.color));
            }
            painter.extend(shapes);
        });

        // Deciding whether or not to update properties of particles
        if self.particles.len() > 0 && (self.default_radius != self.particles[0].radius || self.default_color != self.particles[0].color) {
            // Updating particle dynamics and properties on a timestep
            for p in &mut self.particles {
                // Updating properties
                p.update_properties(self.default_radius, self.default_color);
            }
        }
        
        // Fixed-step physics loop
        let mut steps = 0u32;
        while self.accumulator >= self.default_phys_dt && steps < self.max_substeps {
            self.physics_step(self.default_phys_dt, window_size);
            self.accumulator -= self.default_phys_dt;
            steps += 1;
        }
        
        // Dropping leftover time to keep UI responsive if we hit the step cap 
        if steps == self.max_substeps {
            self.accumulator = 0.0;
        }

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
            let default_phys_dt = 1.0/30.0;
            let default_g = 800.0;
            let default_radius = 2.0;
            let default_mass = 25.0;
            let default_color = Color32::from_rgb(35, 137, 218);
            let restitution = 0.5;
            let sim = FluidSim::new_empty(default_phys_dt, default_g, default_radius, default_mass, default_color, restitution);
            Ok(Box::new(sim))
        }),
    )
}