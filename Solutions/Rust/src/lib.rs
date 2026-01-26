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
// lib.rs - Library entry point
// =============================================================================
//
// GS.Phy2D est un moteur physique 2D haute performance.
//
// CARACTÉRISTIQUES :
// - Intégration Verlet (stable et efficace)
// - Structure SoA pour performances cache optimales
// - Spatial Hash pour collisions O(n)
// - FastInvSqrt (algorithme Quake III)
// - Support particules, AABB, boîtes statiques, corps rigides
//
// UTILISATION BASIQUE :
// ```rust
// use gs_phy2d::PhyWorld;
//
// let mut world = PhyWorld::new();
// world.set_world_bounds(800.0, 600.0);
// world.set_gravity(0.0, 500.0);
//
// // Ajoute des particules
// world.add_particle(100.0, 100.0, 10.0, false, 1.0, 0.5);
//
// // Boucle de simulation
// loop {
//     world.step(1.0 / 60.0);
//     // ... rendu ...
// }
// ```
//
// =============================================================================

// Déclaration des modules
pub mod gs_phy_vec2;
pub mod gs_phy_types;
pub mod gs_phy_aabb;
pub mod gs_phy_spatial_hash;
pub mod gs_phy_world;
pub mod gs_phy_renderer;

// Ré-exportations pour un accès facile
// Permet d'écrire `use gs_phy2d::PhyWorld;` au lieu de
// `use gs_phy2d::gs_phy_world::PhyWorld;`

pub use gs_phy_vec2::{Vec2, Vec3, vec2, vec3, fast_inv_sqrt};
pub use gs_phy_types::{
    Particle, Constraint, StaticBox, RigidBody,
    PHY_FLAG_FIXED, PHY_FLAG_COLLIDABLE,
    TAG_PARTICLE, TAG_AABB,
    encode_tag, decode_tag,
};
pub use gs_phy_aabb::{DynamicAABB, AABBSoA};
pub use gs_phy_spatial_hash::{SpatialHash, BoxGrid};
pub use gs_phy_world::PhyWorld;
pub use gs_phy_renderer::{PhyColor, colors, velocity_to_color, color_to_rgba};

// =============================================================================
// Prélude - imports communs en une seule ligne
// =============================================================================
//
// Permet d'écrire :
// ```rust
// use gs_phy2d::prelude::*;
// ```
// Pour importer tout ce qui est couramment utilisé.
//
// =============================================================================

pub mod prelude {
    pub use crate::gs_phy_vec2::{Vec2, Vec3, vec2, vec3};
    pub use crate::gs_phy_types::{Particle, Constraint, StaticBox, RigidBody};
    pub use crate::gs_phy_aabb::DynamicAABB;
    pub use crate::gs_phy_world::PhyWorld;
    pub use crate::gs_phy_renderer::{PhyColor, colors, velocity_to_color};
}
