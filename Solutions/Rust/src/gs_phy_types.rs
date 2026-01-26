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
// gs_phy_types.rs - Core physics types
// =============================================================================
//
// Ce module définit les structures de données fondamentales :
// - Particle : particule physique (cercle)
// - Constraint : contrainte de distance entre deux particules
// - StaticBox : boîte statique (mur, obstacle)
// - RigidBody : groupe de particules formant un corps rigide
//
// CHOIX D'ARCHITECTURE :
// ----------------------
// Le moteur utilise une architecture SoA (Structure of Arrays) plutôt que
// AoS (Array of Structures) pour les performances. Ces types sont utilisés
// principalement pour la création et l'accès externe. Internement, le World
// stocke les données en tableaux séparés.
//
// =============================================================================

use crate::gs_phy_vec2::Vec2;

// =============================================================================
// Constantes de flags
// =============================================================================
//
// Les flags permettent de marquer des propriétés sur les particules/AABB.
// Utiliser des bits plutôt que des booléens permet de stocker plusieurs
// propriétés dans un seul octet (économie de mémoire et cache-friendly).
//

/// Particule fixe (ne bouge pas, masse infinie)
pub const PHY_FLAG_FIXED: u8 = 1;

/// Particule collidable (participe aux collisions)
pub const PHY_FLAG_COLLIDABLE: u8 = 2;

// =============================================================================
// Particle - Particule physique
// =============================================================================
//
// Représente un cercle dans le monde physique.
// Utilise l'intégration Verlet : on stocke la position actuelle ET la position
// précédente. La vélocité est implicite : vel = pos - old_pos.
//
// AVANTAGES DE VERLET :
// - Stabilité numérique supérieure à Euler
// - Pas besoin de stocker la vélocité explicitement
// - Résolution de contraintes plus simple
// - Moins sensible aux grandes forces (pas d'explosion)
//
// =============================================================================

#[derive(Clone, Copy, Debug)]
pub struct Particle {
    /// Position actuelle
    pub pos: Vec2,

    /// Position précédente (pour Verlet)
    /// La vélocité est calculée comme : pos - old_pos
    pub old_pos: Vec2,

    /// Accélération accumulée
    /// Réinitialisée à zéro après chaque pas de simulation
    pub accel: Vec2,

    /// Rayon de collision
    pub radius: f32,

    /// Masse inverse (1/masse)
    /// Utiliser la masse inverse évite les divisions coûteuses
    /// 0 = masse infinie = objet fixe
    pub inv_mass: f32,

    /// Coefficient de restitution (élasticité)
    /// 0 = pas de rebond (plastique)
    /// 1 = rebond parfait (élastique)
    pub restitution: f32,

    /// Flags de propriétés (voir PHY_FLAG_*)
    pub flags: u8,
}

impl Particle {
    /// Crée une nouvelle particule
    ///
    /// # Arguments
    /// * `x`, `y` - Position initiale
    /// * `radius` - Rayon de collision
    /// * `fixed` - Si true, la particule ne bougera pas
    /// * `mass` - Masse de la particule (ignorée si fixed=true)
    /// * `restitution` - Coefficient de rebond (0.0 à 1.0)
    pub fn new(
        x: f32,
        y: f32,
        radius: f32,
        fixed: bool,
        mass: f32,
        restitution: f32,
    ) -> Self {
        let pos = Vec2::new(x, y);

        // Calcul de la masse inverse
        // Si fixé ou masse <= 0, inv_mass = 0 (masse infinie)
        let inv_mass = if fixed || mass <= 0.0 {
            0.0
        } else {
            1.0 / mass
        };

        // Construction des flags
        let mut flags = PHY_FLAG_COLLIDABLE;
        if fixed {
            flags |= PHY_FLAG_FIXED;
        }

        Self {
            pos,
            old_pos: pos,  // Même position = vélocité initiale nulle
            accel: Vec2::ZERO,
            radius,
            inv_mass,
            restitution,
            flags,
        }
    }

    /// Vérifie si la particule est fixe
    #[inline]
    pub fn is_fixed(&self) -> bool {
        (self.flags & PHY_FLAG_FIXED) != 0
    }

    /// Vérifie si la particule est collidable
    #[inline]
    pub fn is_collidable(&self) -> bool {
        (self.flags & PHY_FLAG_COLLIDABLE) != 0
    }

    /// Calcule la vélocité implicite (depuis Verlet)
    #[inline]
    pub fn velocity(&self) -> Vec2 {
        self.pos - self.old_pos
    }
}

impl Default for Particle {
    fn default() -> Self {
        Self::new(0.0, 0.0, 5.0, false, 1.0, 0.5)
    }
}

// =============================================================================
// Constraint - Contrainte de distance
// =============================================================================
//
// Maintient deux particules à une distance fixe l'une de l'autre.
// Utilisé pour créer des corps rigides (rectangles, formes complexes).
//
// FONCTIONNEMENT :
// À chaque pas de simulation, on mesure la distance entre P1 et P2.
// Si elle diffère de rest_length, on rapproche ou éloigne les particules.
//
// La stiffness contrôle la "dureté" de la contrainte :
// - 1.0 = rigide (corrigé complètement à chaque frame)
// - 0.5 = semi-rigide (corrigé à moitié, plus élastique)
// - 0.1 = très élastique (comme un ressort mou)
//
// =============================================================================

#[derive(Clone, Copy, Debug)]
pub struct Constraint {
    /// Index de la première particule
    pub p1: usize,

    /// Index de la deuxième particule
    pub p2: usize,

    /// Distance de repos (distance à maintenir)
    pub rest_length: f32,

    /// Raideur de la contrainte (0.0 à 1.0)
    pub stiffness: f32,
}

impl Constraint {
    /// Crée une nouvelle contrainte
    ///
    /// Note : la rest_length est généralement calculée automatiquement
    /// par World::add_constraint() à partir des positions des particules.
    pub fn new(p1: usize, p2: usize, rest_length: f32, stiffness: f32) -> Self {
        Self {
            p1,
            p2,
            rest_length,
            stiffness,
        }
    }
}

// =============================================================================
// StaticBox - Boîte statique (mur, obstacle)
// =============================================================================
//
// Rectangle fixe qui ne bouge pas. Les particules et AABB rebondissent dessus.
// Défini par ses bornes min/max plutôt que centre + dimensions pour optimiser
// les tests de collision.
//
// =============================================================================

#[derive(Clone, Copy, Debug)]
pub struct StaticBox {
    /// Coordonnée X minimale (bord gauche)
    pub min_x: f32,

    /// Coordonnée Y minimale (bord haut)
    pub min_y: f32,

    /// Coordonnée X maximale (bord droit)
    pub max_x: f32,

    /// Coordonnée Y maximale (bord bas)
    pub max_y: f32,

    /// Coefficient de restitution
    pub restitution: f32,
}

impl StaticBox {
    /// Crée une boîte statique à partir du centre et des dimensions
    ///
    /// # Arguments
    /// * `x`, `y` - Centre de la boîte
    /// * `width`, `height` - Dimensions totales
    /// * `restitution` - Coefficient de rebond
    pub fn new(x: f32, y: f32, width: f32, height: f32, restitution: f32) -> Self {
        let half_w = width * 0.5;
        let half_h = height * 0.5;
        Self {
            min_x: x - half_w,
            max_x: x + half_w,
            min_y: y - half_h,
            max_y: y + half_h,
            restitution,
        }
    }

    /// Crée une boîte à partir des bornes directement
    pub fn from_bounds(min_x: f32, min_y: f32, max_x: f32, max_y: f32, restitution: f32) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
            restitution,
        }
    }

    /// Largeur de la boîte
    #[inline]
    pub fn width(&self) -> f32 {
        self.max_x - self.min_x
    }

    /// Hauteur de la boîte
    #[inline]
    pub fn height(&self) -> f32 {
        self.max_y - self.min_y
    }

    /// Centre de la boîte
    #[inline]
    pub fn center(&self) -> Vec2 {
        Vec2::new(
            (self.min_x + self.max_x) * 0.5,
            (self.min_y + self.max_y) * 0.5,
        )
    }
}

// =============================================================================
// RigidBody - Corps rigide
// =============================================================================
//
// Un corps rigide est un groupe de particules connectées par des contraintes.
// Il permet de créer des formes complexes qui se comportent comme un seul objet.
//
// EXEMPLE : Rectangle rigide
// --------------------------
//   P0 -------- P1
//   |  \    /   |
//   |   \  /    |
//   |    \/     |
//   |    /\     |
//   |   /  \    |
//   |  /    \   |
//   P3 -------- P2
//
// 4 particules aux coins + 6 contraintes (4 côtés + 2 diagonales)
// Les diagonales empêchent la déformation du rectangle.
//
// =============================================================================

#[derive(Clone, Copy, Debug)]
pub struct RigidBody {
    /// Index de la première particule du corps
    pub particle_start: usize,

    /// Nombre de particules dans ce corps
    pub particle_count: usize,

    /// Index de la première contrainte
    pub constraint_start: usize,

    /// Nombre de contraintes
    pub constraint_count: usize,
}

impl RigidBody {
    pub fn new(
        particle_start: usize,
        particle_count: usize,
        constraint_start: usize,
        constraint_count: usize,
    ) -> Self {
        Self {
            particle_start,
            particle_count,
            constraint_start,
            constraint_count,
        }
    }
}

// =============================================================================
// Tags pour le Spatial Hash
// =============================================================================
//
// Le spatial hash stocke différents types d'objets (particules, AABB).
// Pour les distinguer, on encode le type et l'index dans un seul usize.
//
// Format (64 bits) :
// [3 bits: type][61 bits: index]
//
// Cela permet de stocker jusqu'à 2^61 objets par type (largement suffisant).
//
// =============================================================================

/// Tag pour les particules
pub const TAG_PARTICLE: usize = 0;

/// Tag pour les AABB dynamiques
pub const TAG_AABB: usize = 1;

/// Décalage de bits pour le tag (61 bits pour l'index)
pub const TAG_SHIFT: usize = 61;

/// Masque pour le tag (3 bits de poids fort)
pub const TAG_MASK: usize = 0xE000000000000000;

/// Masque pour l'index (61 bits de poids faible)
pub const INDEX_MASK: usize = 0x1FFFFFFFFFFFFFFF;

/// Encode un type d'objet et son index en un seul usize
#[inline]
pub fn encode_tag(obj_type: usize, index: usize) -> usize {
    (obj_type << TAG_SHIFT) | index
}

/// Décode un tag en type d'objet et index
#[inline]
pub fn decode_tag(tag: usize) -> (usize, usize) {
    let obj_type = (tag & TAG_MASK) >> TAG_SHIFT;
    let index = tag & INDEX_MASK;
    (obj_type, index)
}

// =============================================================================
// Tests unitaires
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_particle_creation() {
        let p = Particle::new(100.0, 200.0, 10.0, false, 2.0, 0.5);
        assert_eq!(p.pos.x, 100.0);
        assert_eq!(p.pos.y, 200.0);
        assert_eq!(p.radius, 10.0);
        assert!((p.inv_mass - 0.5).abs() < 0.0001);
        assert!(!p.is_fixed());
        assert!(p.is_collidable());
    }

    #[test]
    fn test_fixed_particle() {
        let p = Particle::new(0.0, 0.0, 5.0, true, 1.0, 0.5);
        assert!(p.is_fixed());
        assert_eq!(p.inv_mass, 0.0);  // Masse infinie
    }

    #[test]
    fn test_static_box() {
        let b = StaticBox::new(100.0, 100.0, 50.0, 30.0, 0.3);
        assert_eq!(b.min_x, 75.0);
        assert_eq!(b.max_x, 125.0);
        assert_eq!(b.min_y, 85.0);
        assert_eq!(b.max_y, 115.0);
        assert_eq!(b.width(), 50.0);
        assert_eq!(b.height(), 30.0);
    }

    #[test]
    fn test_tag_encoding() {
        let tag = encode_tag(TAG_PARTICLE, 12345);
        let (obj_type, index) = decode_tag(tag);
        assert_eq!(obj_type, TAG_PARTICLE);
        assert_eq!(index, 12345);

        let tag2 = encode_tag(TAG_AABB, 999);
        let (obj_type2, index2) = decode_tag(tag2);
        assert_eq!(obj_type2, TAG_AABB);
        assert_eq!(index2, 999);
    }
}
