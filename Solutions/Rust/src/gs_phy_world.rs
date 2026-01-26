// ============================================================================
// GS.Phy2D - Open source 2D physics engine
// 2026, Vincent Gsell
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
// Created by Vincent Gsell [https://github.com/VincentGsell]
// ============================================================================

// History
// 20260126 - Rust port created.

// =============================================================================
// gs_phy_world.rs - Main physics engine
// =============================================================================
//
// Ce module est le cœur du moteur physique. Il gère :
// - Les particules (cercles dynamiques)
// - Les AABB dynamiques (rectangles)
// - Les boîtes statiques (murs)
// - Les contraintes de distance
// - Les corps rigides
//
// PIPELINE DE SIMULATION (méthode step()) :
// 1. Intégration Verlet (calcul des nouvelles positions)
// 2. Résolution des contraintes (maintien des distances)
// 3. Résolution des collisions (séparation des objets)
// 4. Collision avec les murs et les boîtes statiques
//
// ARCHITECTURE SoA (Structure of Arrays) :
// Au lieu de stocker [Particle, Particle, Particle, ...]
// on stocke [pos_x, pos_x, pos_x, ...], [pos_y, pos_y, pos_y, ...], etc.
// Cela améliore significativement les performances cache.
//
// =============================================================================

use crate::gs_phy_vec2::{Vec2, Vec3, fast_inv_sqrt, vec2};
use crate::gs_phy_types::*;
use crate::gs_phy_aabb::AABBSoA;
use crate::gs_phy_spatial_hash::{SpatialHash, BoxGrid};

// =============================================================================
// ParticleSoA - Structure of Arrays pour les particules
// =============================================================================

#[derive(Default)]
pub struct ParticleSoA {
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub old_pos_x: Vec<f32>,
    pub old_pos_y: Vec<f32>,
    pub accel_x: Vec<f32>,
    pub accel_y: Vec<f32>,
    pub radius: Vec<f32>,
    pub inv_mass: Vec<f32>,
    pub restitution: Vec<f32>,
    pub flags: Vec<u8>,
    /// ID du corps rigide (-1 = particule libre)
    pub body_id: Vec<i32>,
}

impl ParticleSoA {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            pos_x: Vec::with_capacity(capacity),
            pos_y: Vec::with_capacity(capacity),
            old_pos_x: Vec::with_capacity(capacity),
            old_pos_y: Vec::with_capacity(capacity),
            accel_x: Vec::with_capacity(capacity),
            accel_y: Vec::with_capacity(capacity),
            radius: Vec::with_capacity(capacity),
            inv_mass: Vec::with_capacity(capacity),
            restitution: Vec::with_capacity(capacity),
            flags: Vec::with_capacity(capacity),
            body_id: Vec::with_capacity(capacity),
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.pos_x.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.pos_x.is_empty()
    }

    pub fn clear(&mut self) {
        self.pos_x.clear();
        self.pos_y.clear();
        self.old_pos_x.clear();
        self.old_pos_y.clear();
        self.accel_x.clear();
        self.accel_y.clear();
        self.radius.clear();
        self.inv_mass.clear();
        self.restitution.clear();
        self.flags.clear();
        self.body_id.clear();
    }
}

// =============================================================================
// PhyWorld - Monde physique
// =============================================================================

const INITIAL_CAPACITY: usize = 256;

pub struct PhyWorld {
    // Particules (format SoA)
    particles: ParticleSoA,

    // AABB dynamiques (format SoA)
    aabbs: AABBSoA,

    // Boîtes statiques
    boxes: Vec<StaticBox>,

    // Contraintes
    constraints: Vec<Constraint>,

    // Corps rigides
    rigid_bodies: Vec<RigidBody>,

    // Spatial hash pour les collisions dynamiques
    spatial_hash: SpatialHash,

    // Grille pour les boîtes statiques
    box_grid: BoxGrid,

    // Paramètres du monde
    gravity: Vec2,
    damping: f32,
    world_width: f32,
    world_height: f32,

    // Paramètres de simulation
    collision_iterations: usize,
    constraint_iterations: usize,
    restitution: f32,
}

impl PhyWorld {
    /// Crée un nouveau monde physique
    pub fn new() -> Self {
        let mut world = Self {
            particles: ParticleSoA::with_capacity(INITIAL_CAPACITY),
            aabbs: AABBSoA::with_capacity(INITIAL_CAPACITY),
            boxes: Vec::with_capacity(16),
            constraints: Vec::with_capacity(64),
            rigid_bodies: Vec::with_capacity(16),
            spatial_hash: SpatialHash::new(),
            box_grid: BoxGrid::new(),
            gravity: vec2(0.0, 500.0),
            damping: 0.99,
            world_width: 800.0,
            world_height: 600.0,
            collision_iterations: 2,
            constraint_iterations: 1,
            restitution: 0.3,
        };

        world.spatial_hash.setup(800.0, 600.0, 32.0);
        world
    }

    // =========================================================================
    // Configuration
    // =========================================================================

    /// Définit les limites du monde
    pub fn set_world_bounds(&mut self, width: f32, height: f32) {
        self.world_width = width;
        self.world_height = height;

        // Calcule le rayon max pour configurer le spatial hash
        let mut max_radius = 10.0f32;
        for &r in &self.particles.radius {
            max_radius = max_radius.max(r);
        }

        self.spatial_hash.setup(width, height, max_radius * 2.0);
        self.box_grid.dirty = true;
    }

    /// Définit la gravité
    pub fn set_gravity(&mut self, x: f32, y: f32) {
        self.gravity = vec2(x, y);
    }

    // =========================================================================
    // Ajout d'objets
    // =========================================================================

    /// Ajoute une particule et retourne son index
    pub fn add_particle(
        &mut self,
        x: f32,
        y: f32,
        radius: f32,
        fixed: bool,
        mass: f32,
        restitution: f32,
    ) -> usize {
        let inv_mass = if fixed || mass <= 0.0 { 0.0 } else { 1.0 / mass };

        let mut flags = PHY_FLAG_COLLIDABLE;
        if fixed {
            flags |= PHY_FLAG_FIXED;
        }

        self.particles.pos_x.push(x);
        self.particles.pos_y.push(y);
        self.particles.old_pos_x.push(x);
        self.particles.old_pos_y.push(y);
        self.particles.accel_x.push(0.0);
        self.particles.accel_y.push(0.0);
        self.particles.radius.push(radius);
        self.particles.inv_mass.push(inv_mass);
        self.particles.restitution.push(restitution);
        self.particles.flags.push(flags);
        self.particles.body_id.push(-1); // Particule libre

        self.particles.len() - 1
    }

    /// Ajoute un AABB dynamique
    pub fn add_aabb(
        &mut self,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        fixed: bool,
        mass: f32,
        restitution: f32,
    ) -> usize {
        let inv_mass = if fixed || mass <= 0.0 { 0.0 } else { 1.0 / mass };

        let mut flags = PHY_FLAG_COLLIDABLE;
        if fixed {
            flags |= PHY_FLAG_FIXED;
        }

        self.aabbs.pos_x.push(x);
        self.aabbs.pos_y.push(y);
        self.aabbs.old_pos_x.push(x);
        self.aabbs.old_pos_y.push(y);
        self.aabbs.accel_x.push(0.0);
        self.aabbs.accel_y.push(0.0);
        self.aabbs.half_w.push(width * 0.5);
        self.aabbs.half_h.push(height * 0.5);
        self.aabbs.inv_mass.push(inv_mass);
        self.aabbs.restitution.push(restitution);
        self.aabbs.flags.push(flags);

        self.aabbs.len() - 1
    }

    /// Ajoute une boîte statique
    pub fn add_box(&mut self, x: f32, y: f32, width: f32, height: f32, restitution: f32) -> usize {
        self.boxes.push(StaticBox::new(x, y, width, height, restitution));
        self.box_grid.dirty = true;
        self.boxes.len() - 1
    }

    /// Ajoute une contrainte de distance entre deux particules
    pub fn add_constraint(&mut self, p1: usize, p2: usize, stiffness: f32) -> usize {
        // Calcule la distance actuelle comme distance de repos
        let dx = self.particles.pos_x[p2] - self.particles.pos_x[p1];
        let dy = self.particles.pos_y[p2] - self.particles.pos_y[p1];
        let rest_length = (dx * dx + dy * dy).sqrt();

        self.constraints.push(Constraint::new(p1, p2, rest_length, stiffness));
        self.constraints.len() - 1
    }

    /// Crée un rectangle rigide avec des particules aux coins et sur les côtés
    pub fn add_rigid_rect(
        &mut self,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        particle_radius: f32,
        mass: f32,
        restitution: f32,
        stiffness: f32,
        subdivisions: usize,
    ) -> usize {
        let hw = width / 2.0;
        let hh = height / 2.0;

        // Nombre de particules par côté
        let per_side = 1 + subdivisions;
        let particle_count = 4 * per_side;
        let part_mass = mass / particle_count as f32;

        let mut particle_indices = Vec::with_capacity(particle_count);

        // Crée les particules autour du périmètre
        // Côté haut
        for i in 0..per_side {
            let t = i as f32 / per_side as f32;
            let px = x - hw + t * width;
            let py = y - hh;
            particle_indices.push(self.add_particle(px, py, particle_radius, false, part_mass, restitution));
        }

        // Côté droit
        for i in 0..per_side {
            let t = i as f32 / per_side as f32;
            let px = x + hw;
            let py = y - hh + t * height;
            particle_indices.push(self.add_particle(px, py, particle_radius, false, part_mass, restitution));
        }

        // Côté bas
        for i in 0..per_side {
            let t = i as f32 / per_side as f32;
            let px = x + hw - t * width;
            let py = y + hh;
            particle_indices.push(self.add_particle(px, py, particle_radius, false, part_mass, restitution));
        }

        // Côté gauche
        for i in 0..per_side {
            let t = i as f32 / per_side as f32;
            let px = x - hw;
            let py = y + hh - t * height;
            particle_indices.push(self.add_particle(px, py, particle_radius, false, part_mass, restitution));
        }

        // Contraintes périmétrales
        for i in 0..particle_count {
            let next = (i + 1) % particle_count;
            self.add_constraint(particle_indices[i], particle_indices[next], stiffness);
        }

        // Contraintes croisées (pour rigidité)
        for i in 0..particle_count / 2 {
            let j = i + particle_count / 2;
            self.add_constraint(particle_indices[i], particle_indices[j], stiffness);
        }

        // Diagonales des coins
        self.add_constraint(particle_indices[0], particle_indices[2 * per_side], stiffness);
        self.add_constraint(particle_indices[per_side], particle_indices[3 * per_side], stiffness);

        particle_indices[0]
    }

    /// Crée un corps rigide à partir de cercles
    pub fn add_rigid_body(
        &mut self,
        center_x: f32,
        center_y: f32,
        circles: &[Vec3],  // x, y relatifs, rayon
        mass: f32,
        restitution: f32,
        stiffness: f32,
    ) -> usize {
        if circles.is_empty() {
            return usize::MAX;
        }

        let particle_start = self.particles.len();
        let constraint_start = self.constraints.len();
        let part_mass = mass / circles.len() as f32;
        let body_id = self.rigid_bodies.len() as i32;

        // Crée les particules
        for circle in circles {
            let idx = self.add_particle(
                center_x + circle.x,
                center_y + circle.y,
                circle.z,
                false,
                part_mass,
                restitution,
            );
            self.particles.body_id[idx] = body_id;
        }

        // Crée les contraintes entre cercles qui se touchent
        for i in 0..circles.len() {
            for j in (i + 1)..circles.len() {
                let dx = circles[j].x - circles[i].x;
                let dy = circles[j].y - circles[i].y;
                let dist = (dx * dx + dy * dy).sqrt();
                let touch_dist = circles[i].z + circles[j].z;

                // Connecte si les cercles se touchent (avec marge de 10%)
                if dist <= touch_dist * 1.1 {
                    self.add_constraint(particle_start + i, particle_start + j, stiffness);
                }
            }
        }

        // Enregistre le corps rigide
        self.rigid_bodies.push(RigidBody::new(
            particle_start,
            circles.len(),
            constraint_start,
            self.constraints.len() - constraint_start,
        ));

        self.rigid_bodies.len() - 1
    }

    /// Helper : crée un corps rectangulaire à partir de cercles
    pub fn add_rect_body(
        &mut self,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        mass: f32,
        restitution: f32,
    ) -> usize {
        let hw = width / 2.0;
        let hh = height / 2.0;

        let circles = if width > height {
            let main_radius = hh;
            let filler_radius = hh * 0.5;
            vec![
                Vec3::new(-hw + main_radius, 0.0, main_radius),
                Vec3::new(hw - main_radius, 0.0, main_radius),
                Vec3::new(0.0, -hh + filler_radius, filler_radius),
                Vec3::new(0.0, hh - filler_radius, filler_radius),
                Vec3::new(-hw + filler_radius, -hh + filler_radius, filler_radius),
                Vec3::new(hw - filler_radius, -hh + filler_radius, filler_radius),
            ]
        } else {
            let main_radius = hw;
            vec![
                Vec3::new(0.0, -hh + main_radius, main_radius),
                Vec3::new(0.0, hh - main_radius, main_radius),
            ]
        };

        self.add_rigid_body(x, y, &circles, mass, restitution, 1.0)
    }

    // =========================================================================
    // Simulation
    // =========================================================================

    /// Avance la simulation d'un pas de temps
    pub fn step(&mut self, dt: f32) {
        // 1. Intégration Verlet
        self.integrate(dt);
        self.integrate_aabbs(dt);

        // 2. Itérations de collision
        for _ in 0..self.collision_iterations {
            self.solve_constraints();
            self.solve_collisions();
            self.solve_box_collisions();
            self.solve_aabb_box_collisions();
        }
    }

    /// Intégration Verlet pour les particules
    fn integrate(&mut self, dt: f32) {
        let dt2 = dt * dt;
        let grav_x = self.gravity.x;
        let grav_y = self.gravity.y;
        let damping = self.damping;

        for i in 0..self.particles.len() {
            // Skip les particules fixes
            if (self.particles.flags[i] & PHY_FLAG_FIXED) != 0 {
                continue;
            }

            // Verlet : new_pos = pos + (pos - old_pos) * damping + accel * dt²
            let vel_x = (self.particles.pos_x[i] - self.particles.old_pos_x[i]) * damping;
            let vel_y = (self.particles.pos_y[i] - self.particles.old_pos_y[i]) * damping;

            // Ajoute la gravité
            self.particles.accel_x[i] += grav_x;
            self.particles.accel_y[i] += grav_y;

            let new_pos_x = self.particles.pos_x[i] + vel_x + self.particles.accel_x[i] * dt2;
            let new_pos_y = self.particles.pos_y[i] + vel_y + self.particles.accel_y[i] * dt2;

            // Met à jour les positions
            self.particles.old_pos_x[i] = self.particles.pos_x[i];
            self.particles.old_pos_y[i] = self.particles.pos_y[i];
            self.particles.pos_x[i] = new_pos_x;
            self.particles.pos_y[i] = new_pos_y;

            // Reset accélération
            self.particles.accel_x[i] = 0.0;
            self.particles.accel_y[i] = 0.0;
        }
    }

    /// Intégration Verlet pour les AABB
    fn integrate_aabbs(&mut self, dt: f32) {
        let dt2 = dt * dt;
        let grav_x = self.gravity.x;
        let grav_y = self.gravity.y;
        let damping = self.damping;

        for i in 0..self.aabbs.len() {
            if (self.aabbs.flags[i] & PHY_FLAG_FIXED) != 0 {
                continue;
            }

            let vel_x = (self.aabbs.pos_x[i] - self.aabbs.old_pos_x[i]) * damping;
            let vel_y = (self.aabbs.pos_y[i] - self.aabbs.old_pos_y[i]) * damping;

            self.aabbs.accel_x[i] += grav_x;
            self.aabbs.accel_y[i] += grav_y;

            let new_pos_x = self.aabbs.pos_x[i] + vel_x + self.aabbs.accel_x[i] * dt2;
            let new_pos_y = self.aabbs.pos_y[i] + vel_y + self.aabbs.accel_y[i] * dt2;

            self.aabbs.old_pos_x[i] = self.aabbs.pos_x[i];
            self.aabbs.old_pos_y[i] = self.aabbs.pos_y[i];
            self.aabbs.pos_x[i] = new_pos_x;
            self.aabbs.pos_y[i] = new_pos_y;

            self.aabbs.accel_x[i] = 0.0;
            self.aabbs.accel_y[i] = 0.0;
        }
    }

    /// Résout les contraintes de distance
    fn solve_constraints(&mut self) {
        for i in 0..self.constraints.len() {
            let p1 = self.constraints[i].p1;
            let p2 = self.constraints[i].p2;
            let stiffness = self.constraints[i].stiffness;
            let rest_length = self.constraints[i].rest_length;

            let delta_x = self.particles.pos_x[p2] - self.particles.pos_x[p1];
            let delta_y = self.particles.pos_y[p2] - self.particles.pos_y[p1];
            let dist_sq = delta_x * delta_x + delta_y * delta_y;

            if dist_sq < 0.0001 {
                continue;
            }

            let inv_dist = fast_inv_sqrt(dist_sq);
            let dist = dist_sq * inv_dist;
            let diff = (dist - rest_length) * inv_dist;
            let corr_x = delta_x * diff * 0.5 * stiffness;
            let corr_y = delta_y * diff * 0.5 * stiffness;

            let inv_mass1 = self.particles.inv_mass[p1];
            let inv_mass2 = self.particles.inv_mass[p2];
            let total_inv_mass = inv_mass1 + inv_mass2;

            if total_inv_mass < 0.0001 {
                continue;
            }

            let ratio1 = inv_mass1 / total_inv_mass;
            let ratio2 = inv_mass2 / total_inv_mass;

            self.particles.pos_x[p1] += corr_x * ratio1 * 2.0;
            self.particles.pos_y[p1] += corr_y * ratio1 * 2.0;
            self.particles.pos_x[p2] -= corr_x * ratio2 * 2.0;
            self.particles.pos_y[p2] -= corr_y * ratio2 * 2.0;
        }
    }

    /// Résout les collisions entre objets dynamiques
    fn solve_collisions(&mut self) {
        // Remplit le spatial hash
        self.spatial_hash.clear();

        // Insère les particules
        for i in 0..self.particles.len() {
            if (self.particles.flags[i] & PHY_FLAG_COLLIDABLE) != 0 {
                let tag = encode_tag(TAG_PARTICLE, i);
                self.spatial_hash.insert(
                    tag,
                    self.particles.pos_x[i],
                    self.particles.pos_y[i],
                    self.particles.radius[i],
                );
            }
        }

        // Insère les AABB
        for i in 0..self.aabbs.len() {
            if (self.aabbs.flags[i] & PHY_FLAG_COLLIDABLE) != 0 {
                let tag = encode_tag(TAG_AABB, i);
                let rad = self.aabbs.half_w[i].max(self.aabbs.half_h[i]);
                self.spatial_hash.insert(
                    tag,
                    self.aabbs.pos_x[i],
                    self.aabbs.pos_y[i],
                    rad,
                );
            }
        }

        // Queries depuis les particules
        for i in 0..self.particles.len() {
            if (self.particles.flags[i] & PHY_FLAG_COLLIDABLE) == 0 {
                continue;
            }

            let self_tag = encode_tag(TAG_PARTICLE, i);
            let query_count = self.spatial_hash.query(
                self_tag,
                self.particles.pos_x[i],
                self.particles.pos_y[i],
                self.particles.radius[i],
            );

            for k in 0..query_count {
                let tag = self.spatial_hash.get_query_result(k);
                let (obj_type, obj_index) = decode_tag(tag);

                match obj_type {
                    TAG_PARTICLE => self.collide_particles(i, obj_index),
                    TAG_AABB => self.collide_particle_aabb(i, obj_index),
                    _ => {}
                }
            }
        }

        // Queries depuis les AABB
        for i in 0..self.aabbs.len() {
            if (self.aabbs.flags[i] & PHY_FLAG_COLLIDABLE) == 0 {
                continue;
            }

            let self_tag = encode_tag(TAG_AABB, i);
            let rad = self.aabbs.half_w[i].max(self.aabbs.half_h[i]);
            let query_count = self.spatial_hash.query(
                self_tag,
                self.aabbs.pos_x[i],
                self.aabbs.pos_y[i],
                rad,
            );

            for k in 0..query_count {
                let tag = self.spatial_hash.get_query_result(k);
                let (obj_type, obj_index) = decode_tag(tag);

                if obj_type == TAG_AABB {
                    self.collide_aabbs(i, obj_index);
                }
            }
        }
    }

    /// Collision entre deux particules
    fn collide_particles(&mut self, i: usize, j: usize) {
        // Skip si même corps rigide
        let body_i = self.particles.body_id[i];
        let body_j = self.particles.body_id[j];
        if body_i >= 0 && body_i == body_j {
            return;
        }

        let delta_x = self.particles.pos_x[j] - self.particles.pos_x[i];
        let delta_y = self.particles.pos_y[j] - self.particles.pos_y[i];
        let dist_sq = delta_x * delta_x + delta_y * delta_y;
        let min_dist = self.particles.radius[i] + self.particles.radius[j];

        if dist_sq >= min_dist * min_dist {
            return;
        }

        // Gère le cas dist = 0
        let (delta_x, delta_y, dist_sq) = if dist_sq < 0.0001 {
            (0.1, 0.1, 0.02)
        } else {
            (delta_x, delta_y, dist_sq)
        };

        let inv_dist = fast_inv_sqrt(dist_sq);
        let dist = dist_sq * inv_dist;
        let overlap = min_dist - dist;
        let normal_x = delta_x * inv_dist;
        let normal_y = delta_y * inv_dist;

        let inv_mass1 = self.particles.inv_mass[i];
        let inv_mass2 = self.particles.inv_mass[j];
        let total_inv_mass = inv_mass1 + inv_mass2;

        if total_inv_mass < 0.0001 {
            return;
        }

        let ratio1 = inv_mass1 / total_inv_mass;
        let ratio2 = inv_mass2 / total_inv_mass;

        let corr_x = normal_x * overlap;
        let corr_y = normal_y * overlap;

        // Sépare les particules
        self.particles.pos_x[i] -= corr_x * ratio1;
        self.particles.pos_y[i] -= corr_y * ratio1;
        self.particles.pos_x[j] += corr_x * ratio2;
        self.particles.pos_y[j] += corr_y * ratio2;

        // Applique l'impulsion (rebond)
        let v1x = self.particles.pos_x[i] - self.particles.old_pos_x[i];
        let v1y = self.particles.pos_y[i] - self.particles.old_pos_y[i];
        let v2x = self.particles.pos_x[j] - self.particles.old_pos_x[j];
        let v2y = self.particles.pos_y[j] - self.particles.old_pos_y[j];

        let rel_vel = (v2x - v1x) * normal_x + (v2y - v1y) * normal_y;

        if rel_vel > 0.0 {
            return;
        }

        let impulse = -(1.0 + self.restitution) * rel_vel / total_inv_mass;
        let imp_x = normal_x * impulse;
        let imp_y = normal_y * impulse;

        self.particles.old_pos_x[i] = self.particles.pos_x[i] - (v1x - imp_x * inv_mass1);
        self.particles.old_pos_y[i] = self.particles.pos_y[i] - (v1y - imp_y * inv_mass1);
        self.particles.old_pos_x[j] = self.particles.pos_x[j] - (v2x + imp_x * inv_mass2);
        self.particles.old_pos_y[j] = self.particles.pos_y[j] - (v2y + imp_y * inv_mass2);
    }

    /// Collision entre deux AABB
    fn collide_aabbs(&mut self, i: usize, j: usize) {
        let min_x1 = self.aabbs.pos_x[i] - self.aabbs.half_w[i];
        let max_x1 = self.aabbs.pos_x[i] + self.aabbs.half_w[i];
        let min_y1 = self.aabbs.pos_y[i] - self.aabbs.half_h[i];
        let max_y1 = self.aabbs.pos_y[i] + self.aabbs.half_h[i];

        let min_x2 = self.aabbs.pos_x[j] - self.aabbs.half_w[j];
        let max_x2 = self.aabbs.pos_x[j] + self.aabbs.half_w[j];
        let min_y2 = self.aabbs.pos_y[j] - self.aabbs.half_h[j];
        let max_y2 = self.aabbs.pos_y[j] + self.aabbs.half_h[j];

        // Test d'overlap
        if max_x1 < min_x2 || min_x1 > max_x2 || max_y1 < min_y2 || min_y1 > max_y2 {
            return;
        }

        // Calcule l'overlap sur chaque axe
        let overlap_x = if self.aabbs.pos_x[i] < self.aabbs.pos_x[j] {
            max_x1 - min_x2
        } else {
            max_x2 - min_x1
        };

        let overlap_y = if self.aabbs.pos_y[i] < self.aabbs.pos_y[j] {
            max_y1 - min_y2
        } else {
            max_y2 - min_y1
        };

        // Pousse selon l'axe de moindre overlap
        let (overlap, nx, ny) = if overlap_x < overlap_y {
            if self.aabbs.pos_x[i] < self.aabbs.pos_x[j] {
                (overlap_x, -1.0, 0.0)
            } else {
                (overlap_x, 1.0, 0.0)
            }
        } else {
            if self.aabbs.pos_y[i] < self.aabbs.pos_y[j] {
                (overlap_y, 0.0, -1.0)
            } else {
                (overlap_y, 0.0, 1.0)
            }
        };

        let inv_mass1 = self.aabbs.inv_mass[i];
        let inv_mass2 = self.aabbs.inv_mass[j];
        let total_inv_mass = inv_mass1 + inv_mass2;

        if total_inv_mass < 0.0001 {
            return;
        }

        let ratio1 = inv_mass1 / total_inv_mass;
        let ratio2 = inv_mass2 / total_inv_mass;

        // Sépare
        self.aabbs.pos_x[i] += nx * overlap * ratio1;
        self.aabbs.pos_y[i] += ny * overlap * ratio1;
        self.aabbs.pos_x[j] -= nx * overlap * ratio2;
        self.aabbs.pos_y[j] -= ny * overlap * ratio2;

        // Impulsion
        let v1x = self.aabbs.pos_x[i] - self.aabbs.old_pos_x[i];
        let v1y = self.aabbs.pos_y[i] - self.aabbs.old_pos_y[i];
        let v2x = self.aabbs.pos_x[j] - self.aabbs.old_pos_x[j];
        let v2y = self.aabbs.pos_y[j] - self.aabbs.old_pos_y[j];

        let rel_vel = (v1x - v2x) * nx + (v1y - v2y) * ny;

        if rel_vel > 0.0 {
            return;
        }

        let impulse = -(1.0 + self.restitution) * rel_vel / total_inv_mass;
        let imp_x = nx * impulse;
        let imp_y = ny * impulse;

        self.aabbs.old_pos_x[i] = self.aabbs.pos_x[i] - (v1x + imp_x * inv_mass1);
        self.aabbs.old_pos_y[i] = self.aabbs.pos_y[i] - (v1y + imp_y * inv_mass1);
        self.aabbs.old_pos_x[j] = self.aabbs.pos_x[j] - (v2x - imp_x * inv_mass2);
        self.aabbs.old_pos_y[j] = self.aabbs.pos_y[j] - (v2y - imp_y * inv_mass2);
    }

    /// Collision particule-AABB
    fn collide_particle_aabb(&mut self, pi: usize, ai: usize) {
        let pos_x = self.particles.pos_x[pi];
        let pos_y = self.particles.pos_y[pi];
        let rad = self.particles.radius[pi];

        let min_x = self.aabbs.pos_x[ai] - self.aabbs.half_w[ai];
        let max_x = self.aabbs.pos_x[ai] + self.aabbs.half_w[ai];
        let min_y = self.aabbs.pos_y[ai] - self.aabbs.half_h[ai];
        let max_y = self.aabbs.pos_y[ai] + self.aabbs.half_h[ai];

        // Point le plus proche sur l'AABB
        let closest_x = pos_x.clamp(min_x, max_x);
        let closest_y = pos_y.clamp(min_y, max_y);

        let dx = pos_x - closest_x;
        let dy = pos_y - closest_y;
        let dist_sq = dx * dx + dy * dy;

        if dist_sq >= rad * rad {
            return;
        }

        let inv_mass_p = self.particles.inv_mass[pi];
        let inv_mass_a = self.aabbs.inv_mass[ai];
        let total_inv_mass = inv_mass_p + inv_mass_a;

        if total_inv_mass < 0.0001 {
            return;
        }

        let (nx, ny, overlap) = if dist_sq < 0.0001 {
            // Particule à l'intérieur de l'AABB
            let dx_center = pos_x - self.aabbs.pos_x[ai];
            let dy_center = pos_y - self.aabbs.pos_y[ai];
            if dx_center.abs() > dy_center.abs() {
                if dx_center > 0.0 {
                    (1.0, 0.0, max_x - pos_x + rad)
                } else {
                    (-1.0, 0.0, pos_x - min_x + rad)
                }
            } else {
                if dy_center > 0.0 {
                    (0.0, 1.0, max_y - pos_y + rad)
                } else {
                    (0.0, -1.0, pos_y - min_y + rad)
                }
            }
        } else {
            let inv_dist = fast_inv_sqrt(dist_sq);
            let dist = dist_sq * inv_dist;
            (dx * inv_dist, dy * inv_dist, rad - dist)
        };

        let ratio1 = inv_mass_p / total_inv_mass;
        let ratio2 = inv_mass_a / total_inv_mass;

        // Sépare
        self.particles.pos_x[pi] += nx * overlap * ratio1;
        self.particles.pos_y[pi] += ny * overlap * ratio1;
        self.aabbs.pos_x[ai] -= nx * overlap * ratio2;
        self.aabbs.pos_y[ai] -= ny * overlap * ratio2;

        // Impulsion
        let vpx = self.particles.pos_x[pi] - self.particles.old_pos_x[pi];
        let vpy = self.particles.pos_y[pi] - self.particles.old_pos_y[pi];
        let vax = self.aabbs.pos_x[ai] - self.aabbs.old_pos_x[ai];
        let vay = self.aabbs.pos_y[ai] - self.aabbs.old_pos_y[ai];

        let rel_vel = (vpx - vax) * nx + (vpy - vay) * ny;

        if rel_vel > 0.0 {
            return;
        }

        let impulse = -(1.0 + self.restitution) * rel_vel / total_inv_mass;
        let imp_x = nx * impulse;
        let imp_y = ny * impulse;

        self.particles.old_pos_x[pi] = self.particles.pos_x[pi] - (vpx + imp_x * inv_mass_p);
        self.particles.old_pos_y[pi] = self.particles.pos_y[pi] - (vpy + imp_y * inv_mass_p);
        self.aabbs.old_pos_x[ai] = self.aabbs.pos_x[ai] - (vax - imp_x * inv_mass_a);
        self.aabbs.old_pos_y[ai] = self.aabbs.pos_y[ai] - (vay - imp_y * inv_mass_a);
    }

    /// Collision particules avec boîtes statiques et bords du monde
    fn solve_box_collisions(&mut self) {
        if self.box_grid.dirty {
            self.box_grid.rebuild(self.world_width, self.world_height, &self.boxes);
        }

        let bounds_right = self.world_width;
        let bounds_bottom = self.world_height;

        for i in 0..self.particles.len() {
            if (self.particles.flags[i] & PHY_FLAG_FIXED) != 0 {
                continue;
            }
            if (self.particles.flags[i] & PHY_FLAG_COLLIDABLE) == 0 {
                continue;
            }

            let mut pos_x = self.particles.pos_x[i];
            let mut pos_y = self.particles.pos_y[i];
            let rad = self.particles.radius[i];

            // Bords du monde
            if pos_x < rad {
                let vel_x = pos_x - self.particles.old_pos_x[i];
                pos_x = rad;
                self.particles.old_pos_x[i] = if vel_x < 0.0 {
                    pos_x + vel_x * self.restitution
                } else {
                    pos_x - vel_x
                };
                self.particles.pos_x[i] = pos_x;
            } else if pos_x > bounds_right - rad {
                let vel_x = pos_x - self.particles.old_pos_x[i];
                pos_x = bounds_right - rad;
                self.particles.old_pos_x[i] = if vel_x > 0.0 {
                    pos_x + vel_x * self.restitution
                } else {
                    pos_x - vel_x
                };
                self.particles.pos_x[i] = pos_x;
            }

            if pos_y < rad {
                let vel_y = pos_y - self.particles.old_pos_y[i];
                pos_y = rad;
                self.particles.old_pos_y[i] = if vel_y < 0.0 {
                    pos_y + vel_y * self.restitution
                } else {
                    pos_y - vel_y
                };
                self.particles.pos_y[i] = pos_y;
            } else if pos_y > bounds_bottom - rad {
                let vel_y = pos_y - self.particles.old_pos_y[i];
                pos_y = bounds_bottom - rad;
                self.particles.old_pos_y[i] = if vel_y > 0.0 {
                    pos_y + vel_y * self.restitution
                } else {
                    pos_y - vel_y
                };
                self.particles.pos_y[i] = pos_y;
            }

            // Collision avec boîtes statiques
            // Note: On copie les indices pour éviter le conflit de borrow
            if !self.boxes.is_empty() {
                if let Some(cell) = self.box_grid.get_cell(pos_x, pos_y) {
                    // Copie des indices dans un buffer temporaire
                    let count = cell.count;
                    let mut box_indices = [0usize; 8]; // Même taille que BOX_GRID_MAX_PER_CELL
                    for k in 0..count {
                        box_indices[k] = cell.box_indices[k];
                    }
                    // Maintenant on peut emprunter self mutablement
                    for k in 0..count {
                        self.collide_particle_static_box(i, box_indices[k]);
                    }
                }
            }
        }
    }

    /// Collision particule-boîte statique
    fn collide_particle_static_box(&mut self, i: usize, box_idx: usize) {
        let b = &self.boxes[box_idx];
        let pos_x = self.particles.pos_x[i];
        let pos_y = self.particles.pos_y[i];
        let rad = self.particles.radius[i];

        // Point le plus proche
        let closest_x = pos_x.clamp(b.min_x, b.max_x);
        let closest_y = pos_y.clamp(b.min_y, b.max_y);

        let dx = pos_x - closest_x;
        let dy = pos_y - closest_y;
        let dist_sq = dx * dx + dy * dy;

        if dist_sq >= rad * rad {
            return;
        }

        let vel_x = pos_x - self.particles.old_pos_x[i];
        let vel_y = pos_y - self.particles.old_pos_y[i];

        let (new_pos_x, new_pos_y, nx, ny) = if dist_sq < 0.0001 {
            // À l'intérieur - pousse vers le bord le plus proche
            let edge_left = pos_x - b.min_x;
            let edge_right = b.max_x - pos_x;
            let edge_top = pos_y - b.min_y;
            let edge_bottom = b.max_y - pos_y;

            let min_edge = edge_left.min(edge_right).min(edge_top).min(edge_bottom);

            if min_edge == edge_left {
                (pos_x - (edge_left + rad), pos_y, -1.0, 0.0)
            } else if min_edge == edge_right {
                (pos_x + (edge_right + rad), pos_y, 1.0, 0.0)
            } else if min_edge == edge_top {
                (pos_x, pos_y - (edge_top + rad), 0.0, -1.0)
            } else {
                (pos_x, pos_y + (edge_bottom + rad), 0.0, 1.0)
            }
        } else {
            let inv_dist = fast_inv_sqrt(dist_sq);
            let dist = dist_sq * inv_dist;
            let overlap = rad - dist;
            let nx = dx * inv_dist;
            let ny = dy * inv_dist;
            (pos_x + nx * overlap, pos_y + ny * overlap, nx, ny)
        };

        self.particles.pos_x[i] = new_pos_x;
        self.particles.pos_y[i] = new_pos_y;

        // Vélocité le long de la normale
        let vel_n = vel_x * nx + vel_y * ny;

        if vel_n < 0.0 {
            let impulse = (1.0 + self.restitution) * vel_n;
            self.particles.old_pos_x[i] = new_pos_x - (vel_x - impulse * nx);
            self.particles.old_pos_y[i] = new_pos_y - (vel_y - impulse * ny);
        } else {
            self.particles.old_pos_x[i] = new_pos_x - vel_x;
            self.particles.old_pos_y[i] = new_pos_y - vel_y;
        }
    }

    /// Collision AABB avec bords du monde et boîtes statiques
    fn solve_aabb_box_collisions(&mut self) {
        if self.box_grid.dirty {
            self.box_grid.rebuild(self.world_width, self.world_height, &self.boxes);
        }

        let bounds_right = self.world_width;
        let bounds_bottom = self.world_height;

        for i in 0..self.aabbs.len() {
            if (self.aabbs.flags[i] & PHY_FLAG_FIXED) != 0 {
                continue;
            }
            if (self.aabbs.flags[i] & PHY_FLAG_COLLIDABLE) == 0 {
                continue;
            }

            let mut pos_x = self.aabbs.pos_x[i];
            let mut pos_y = self.aabbs.pos_y[i];
            let hw = self.aabbs.half_w[i];
            let hh = self.aabbs.half_h[i];

            // Bords
            if pos_x - hw < 0.0 {
                let vel_x = pos_x - self.aabbs.old_pos_x[i];
                pos_x = hw;
                self.aabbs.old_pos_x[i] = if vel_x < 0.0 {
                    pos_x + vel_x * self.restitution
                } else {
                    pos_x - vel_x
                };
                self.aabbs.pos_x[i] = pos_x;
            } else if pos_x + hw > bounds_right {
                let vel_x = pos_x - self.aabbs.old_pos_x[i];
                pos_x = bounds_right - hw;
                self.aabbs.old_pos_x[i] = if vel_x > 0.0 {
                    pos_x + vel_x * self.restitution
                } else {
                    pos_x - vel_x
                };
                self.aabbs.pos_x[i] = pos_x;
            }

            if pos_y - hh < 0.0 {
                let vel_y = pos_y - self.aabbs.old_pos_y[i];
                pos_y = hh;
                self.aabbs.old_pos_y[i] = if vel_y < 0.0 {
                    pos_y + vel_y * self.restitution
                } else {
                    pos_y - vel_y
                };
                self.aabbs.pos_y[i] = pos_y;
            } else if pos_y + hh > bounds_bottom {
                let vel_y = pos_y - self.aabbs.old_pos_y[i];
                pos_y = bounds_bottom - hh;
                self.aabbs.old_pos_y[i] = if vel_y > 0.0 {
                    pos_y + vel_y * self.restitution
                } else {
                    pos_y - vel_y
                };
                self.aabbs.pos_y[i] = pos_y;
            }
        }
    }

    /// Vide le monde
    pub fn clear(&mut self) {
        self.particles.clear();
        self.aabbs.clear();
        self.boxes.clear();
        self.constraints.clear();
        self.rigid_bodies.clear();
        self.box_grid.dirty = true;
    }

    // =========================================================================
    // Accesseurs
    // =========================================================================

    pub fn particle_count(&self) -> usize { self.particles.len() }
    pub fn aabb_count(&self) -> usize { self.aabbs.len() }
    pub fn box_count(&self) -> usize { self.boxes.len() }
    pub fn constraint_count(&self) -> usize { self.constraints.len() }
    pub fn rigid_body_count(&self) -> usize { self.rigid_bodies.len() }

    // Accès direct SoA pour le rendu
    pub fn get_pos_x(&self, i: usize) -> f32 { self.particles.pos_x[i] }
    pub fn get_pos_y(&self, i: usize) -> f32 { self.particles.pos_y[i] }
    pub fn get_old_pos_x(&self, i: usize) -> f32 { self.particles.old_pos_x[i] }
    pub fn get_old_pos_y(&self, i: usize) -> f32 { self.particles.old_pos_y[i] }
    pub fn get_radius(&self, i: usize) -> f32 { self.particles.radius[i] }

    pub fn get_aabb_pos_x(&self, i: usize) -> f32 { self.aabbs.pos_x[i] }
    pub fn get_aabb_pos_y(&self, i: usize) -> f32 { self.aabbs.pos_y[i] }
    pub fn get_aabb_old_pos_x(&self, i: usize) -> f32 { self.aabbs.old_pos_x[i] }
    pub fn get_aabb_old_pos_y(&self, i: usize) -> f32 { self.aabbs.old_pos_y[i] }
    pub fn get_aabb_half_w(&self, i: usize) -> f32 { self.aabbs.half_w[i] }
    pub fn get_aabb_half_h(&self, i: usize) -> f32 { self.aabbs.half_h[i] }

    pub fn get_box(&self, i: usize) -> &StaticBox { &self.boxes[i] }
    pub fn get_constraint(&self, i: usize) -> &Constraint { &self.constraints[i] }

    // Setters
    pub fn set_damping(&mut self, v: f32) { self.damping = v; }
    pub fn set_restitution(&mut self, v: f32) { self.restitution = v; }
    pub fn set_collision_iterations(&mut self, v: usize) { self.collision_iterations = v; }

    pub fn damping(&self) -> f32 { self.damping }
    pub fn restitution(&self) -> f32 { self.restitution }
    pub fn collision_iterations(&self) -> usize { self.collision_iterations }
    pub fn world_width(&self) -> f32 { self.world_width }
    pub fn world_height(&self) -> f32 { self.world_height }
}

impl Default for PhyWorld {
    fn default() -> Self {
        Self::new()
    }
}
