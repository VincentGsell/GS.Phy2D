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
// gs_phy_aabb.rs - Dynamic AABB (Axis-Aligned Bounding Box)
// =============================================================================
//
// Un AABB est une boîte rectangulaire dont les côtés sont alignés avec les axes.
// Contrairement aux OBB (Oriented Bounding Box), les AABB ne peuvent pas tourner.
//
// AVANTAGES DES AABB :
// - Tests de collision très rapides (juste comparaison de bornes)
// - Intégration simple (pas de rotation à gérer)
// - Parfaits pour des objets qui ne tournent pas (plateformes, caisses)
//
// LIMITATIONS :
// - Pas de rotation possible
// - Approximation grossière pour les objets non-rectangulaires
//
// =============================================================================

use crate::gs_phy_types::{PHY_FLAG_FIXED, PHY_FLAG_COLLIDABLE};

// =============================================================================
// DynamicAABB - AABB dynamique (peut bouger)
// =============================================================================
//
// Similaire à Particle mais rectangulaire. Utilise aussi l'intégration Verlet.
//
// =============================================================================

#[derive(Clone, Copy, Debug)]
pub struct DynamicAABB {
    /// Position X du centre
    pub pos_x: f32,
    /// Position Y du centre
    pub pos_y: f32,

    /// Position X précédente (Verlet)
    pub old_pos_x: f32,
    /// Position Y précédente (Verlet)
    pub old_pos_y: f32,

    /// Accélération X accumulée
    pub accel_x: f32,
    /// Accélération Y accumulée
    pub accel_y: f32,

    /// Demi-largeur (distance du centre au bord horizontal)
    pub half_w: f32,
    /// Demi-hauteur (distance du centre au bord vertical)
    pub half_h: f32,

    /// Masse inverse (0 = fixe)
    pub inv_mass: f32,

    /// Coefficient de restitution
    pub restitution: f32,

    /// Flags (PHY_FLAG_FIXED, PHY_FLAG_COLLIDABLE)
    pub flags: u8,
}

impl DynamicAABB {
    /// Crée un nouvel AABB dynamique
    ///
    /// # Arguments
    /// * `x`, `y` - Position du centre
    /// * `width`, `height` - Dimensions totales
    /// * `fixed` - Si true, l'AABB ne bougera pas
    /// * `mass` - Masse
    /// * `restitution` - Coefficient de rebond
    pub fn new(
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        fixed: bool,
        mass: f32,
        restitution: f32,
    ) -> Self {
        let inv_mass = if fixed || mass <= 0.0 {
            0.0
        } else {
            1.0 / mass
        };

        let mut flags = PHY_FLAG_COLLIDABLE;
        if fixed {
            flags |= PHY_FLAG_FIXED;
        }

        Self {
            pos_x: x,
            pos_y: y,
            old_pos_x: x,
            old_pos_y: y,
            accel_x: 0.0,
            accel_y: 0.0,
            half_w: width * 0.5,
            half_h: height * 0.5,
            inv_mass,
            restitution,
            flags,
        }
    }

    /// Coordonnée X minimale (bord gauche)
    #[inline]
    pub fn min_x(&self) -> f32 {
        self.pos_x - self.half_w
    }

    /// Coordonnée X maximale (bord droit)
    #[inline]
    pub fn max_x(&self) -> f32 {
        self.pos_x + self.half_w
    }

    /// Coordonnée Y minimale (bord haut)
    #[inline]
    pub fn min_y(&self) -> f32 {
        self.pos_y - self.half_h
    }

    /// Coordonnée Y maximale (bord bas)
    #[inline]
    pub fn max_y(&self) -> f32 {
        self.pos_y + self.half_h
    }

    /// Largeur totale
    #[inline]
    pub fn width(&self) -> f32 {
        self.half_w * 2.0
    }

    /// Hauteur totale
    #[inline]
    pub fn height(&self) -> f32 {
        self.half_h * 2.0
    }

    /// Vélocité X (Verlet)
    #[inline]
    pub fn vel_x(&self) -> f32 {
        self.pos_x - self.old_pos_x
    }

    /// Vélocité Y (Verlet)
    #[inline]
    pub fn vel_y(&self) -> f32 {
        self.pos_y - self.old_pos_y
    }

    /// Est-ce que l'AABB est fixe ?
    #[inline]
    pub fn is_fixed(&self) -> bool {
        (self.flags & PHY_FLAG_FIXED) != 0
    }

    /// Est-ce que l'AABB est collidable ?
    #[inline]
    pub fn is_collidable(&self) -> bool {
        (self.flags & PHY_FLAG_COLLIDABLE) != 0
    }

    /// Rayon englobant (pour le spatial hash)
    /// Retourne le plus grand des deux demi-axes
    #[inline]
    pub fn bounding_radius(&self) -> f32 {
        if self.half_w > self.half_h {
            self.half_w
        } else {
            self.half_h
        }
    }
}

impl Default for DynamicAABB {
    fn default() -> Self {
        Self::new(0.0, 0.0, 20.0, 20.0, false, 1.0, 0.5)
    }
}

// =============================================================================
// AABBSoA - Structure of Arrays pour les AABB
// =============================================================================
//
// Pour les performances, le World stocke les AABB en format SoA :
// au lieu d'un tableau de structs [AABB, AABB, AABB, ...]
// on a des tableaux séparés [pos_x...], [pos_y...], [half_w...], etc.
//
// AVANTAGES :
// - Meilleur cache locality quand on itère sur une seule propriété
// - Permet la vectorisation SIMD plus facilement
// - Alignement mémoire optimal
//
// =============================================================================

#[derive(Default)]
pub struct AABBSoA {
    /// Positions X des centres
    pub pos_x: Vec<f32>,
    /// Positions Y des centres
    pub pos_y: Vec<f32>,

    /// Positions X précédentes
    pub old_pos_x: Vec<f32>,
    /// Positions Y précédentes
    pub old_pos_y: Vec<f32>,

    /// Accélérations X
    pub accel_x: Vec<f32>,
    /// Accélérations Y
    pub accel_y: Vec<f32>,

    /// Demi-largeurs
    pub half_w: Vec<f32>,
    /// Demi-hauteurs
    pub half_h: Vec<f32>,

    /// Masses inverses
    pub inv_mass: Vec<f32>,

    /// Coefficients de restitution
    pub restitution: Vec<f32>,

    /// Flags
    pub flags: Vec<u8>,
}

impl AABBSoA {
    /// Crée une nouvelle structure SoA avec capacité initiale
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            pos_x: Vec::with_capacity(capacity),
            pos_y: Vec::with_capacity(capacity),
            old_pos_x: Vec::with_capacity(capacity),
            old_pos_y: Vec::with_capacity(capacity),
            accel_x: Vec::with_capacity(capacity),
            accel_y: Vec::with_capacity(capacity),
            half_w: Vec::with_capacity(capacity),
            half_h: Vec::with_capacity(capacity),
            inv_mass: Vec::with_capacity(capacity),
            restitution: Vec::with_capacity(capacity),
            flags: Vec::with_capacity(capacity),
        }
    }

    /// Ajoute un AABB à la structure
    pub fn push(&mut self, aabb: &DynamicAABB) {
        self.pos_x.push(aabb.pos_x);
        self.pos_y.push(aabb.pos_y);
        self.old_pos_x.push(aabb.old_pos_x);
        self.old_pos_y.push(aabb.old_pos_y);
        self.accel_x.push(aabb.accel_x);
        self.accel_y.push(aabb.accel_y);
        self.half_w.push(aabb.half_w);
        self.half_h.push(aabb.half_h);
        self.inv_mass.push(aabb.inv_mass);
        self.restitution.push(aabb.restitution);
        self.flags.push(aabb.flags);
    }

    /// Nombre d'AABB stockés
    #[inline]
    pub fn len(&self) -> usize {
        self.pos_x.len()
    }

    /// Est-ce que la structure est vide ?
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.pos_x.is_empty()
    }

    /// Vide tous les tableaux
    pub fn clear(&mut self) {
        self.pos_x.clear();
        self.pos_y.clear();
        self.old_pos_x.clear();
        self.old_pos_y.clear();
        self.accel_x.clear();
        self.accel_y.clear();
        self.half_w.clear();
        self.half_h.clear();
        self.inv_mass.clear();
        self.restitution.clear();
        self.flags.clear();
    }
}

// =============================================================================
// Tests unitaires
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aabb_creation() {
        let aabb = DynamicAABB::new(100.0, 50.0, 40.0, 20.0, false, 1.0, 0.5);
        assert_eq!(aabb.pos_x, 100.0);
        assert_eq!(aabb.pos_y, 50.0);
        assert_eq!(aabb.half_w, 20.0);
        assert_eq!(aabb.half_h, 10.0);
        assert_eq!(aabb.min_x(), 80.0);
        assert_eq!(aabb.max_x(), 120.0);
        assert_eq!(aabb.min_y(), 40.0);
        assert_eq!(aabb.max_y(), 60.0);
    }

    #[test]
    fn test_aabb_soa() {
        let mut soa = AABBSoA::with_capacity(10);
        assert!(soa.is_empty());

        let aabb = DynamicAABB::new(10.0, 20.0, 30.0, 40.0, false, 1.0, 0.5);
        soa.push(&aabb);

        assert_eq!(soa.len(), 1);
        assert_eq!(soa.pos_x[0], 10.0);
        assert_eq!(soa.pos_y[0], 20.0);
        assert_eq!(soa.half_w[0], 15.0);
        assert_eq!(soa.half_h[0], 20.0);
    }
}
