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
// gs_phy_vec2.rs - 2D/3D vector mathematics
// =============================================================================
//
// Ce module fournit les types et fonctions de base pour les calculs vectoriels :
// - Vec2 : Vecteur 2D (x, y)
// - Vec3 : Vecteur 3D utilisé pour les cercles (x, y, rayon)
// - FastInvSqrt : Algorithme Quake III pour 1/sqrt(x) rapide
//
// OPTIMISATIONS :
// - Toutes les fonctions sont marquées `#[inline]` pour l'inlining
// - Utilisation de `#[derive(Copy)]` pour éviter les allocations
// - FastInvSqrt évite le coût de la division et sqrt standard
//
// =============================================================================

// =============================================================================
// Vec2 - Vecteur 2D
// =============================================================================
//
// Structure fondamentale pour représenter positions, vitesses, accélérations.
// Utilise f32 (32 bits) car suffisant pour la physique 2D et plus cache-friendly.
//
// Les traits dérivés :
// - Clone/Copy : permet la copie implicite (comme les primitifs)
// - Debug : permet l'affichage avec {:?}
// - Default : fournit Vec2 { x: 0.0, y: 0.0 }
// - PartialEq : permet la comparaison avec ==
//
// =============================================================================

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    // Constructeur constant - peut être utilisé dans les constantes
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    // Vecteur nul (0, 0) - constante réutilisable
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    // Addition de deux vecteurs : a + b
    #[inline]
    pub fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    // Soustraction de deux vecteurs : a - b
    #[inline]
    pub fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }

    // Multiplication par un scalaire : v * s
    #[inline]
    pub fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }

    // Produit scalaire (dot product) : a · b
    // Résultat : |a| * |b| * cos(angle)
    // Utile pour projections et calculs d'angles
    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    // Longueur au carré : x² + y²
    // Plus rapide que length() car évite la racine carrée
    // Utile pour comparer des distances sans calcul coûteux
    #[inline]
    pub fn length_sq(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    // Longueur (magnitude) du vecteur : sqrt(x² + y²)
    #[inline]
    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    // Normalisation : retourne un vecteur de longueur 1
    // Si le vecteur est trop petit, retourne ZERO pour éviter division par zéro
    #[inline]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > 0.0001 {
            Self {
                x: self.x / len,
                y: self.y / len,
            }
        } else {
            Self::ZERO
        }
    }

    // Normalisation rapide utilisant FastInvSqrt
    // ~1% moins précis mais significativement plus rapide
    // Idéal pour les calculs de collision en temps réel
    #[inline]
    pub fn normalize_fast(self) -> Self {
        let len_sq = self.x * self.x + self.y * self.y;
        if len_sq > 0.0001 {
            let inv_len = fast_inv_sqrt(len_sq);
            Self {
                x: self.x * inv_len,
                y: self.y * inv_len,
            }
        } else {
            Self::ZERO
        }
    }

    // Distance entre deux points
    #[inline]
    pub fn dist(self, other: Self) -> f32 {
        self.sub(other).length()
    }

    // Distance au carré entre deux points (évite sqrt)
    #[inline]
    pub fn dist_sq(self, other: Self) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dx * dx + dy * dy
    }
}

// =============================================================================
// Implémentation des opérateurs standards
// =============================================================================
// Permet d'utiliser la syntaxe naturelle : vec_a + vec_b, vec * 2.0, etc.

use std::ops::{Add, Sub, Mul, AddAssign, SubAssign};

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

// =============================================================================
// Vec3 - Vecteur 3D (utilisé pour définir des cercles)
// =============================================================================
//
// Dans GS.Phy2D, Vec3 est principalement utilisé pour définir des cercles :
// - x, y : position du centre (relative au corps rigide)
// - z : rayon du cercle
//
// Cette convention permet de passer des tableaux de cercles facilement.
//
// =============================================================================

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

// =============================================================================
// FastInvSqrt - Racine carrée inverse rapide
// =============================================================================
//
// HISTORIQUE :
// Cet algorithme a été popularisé par Quake III Arena (1999), attribué à
// John Carmack mais en réalité développé par plusieurs personnes chez id Software.
//
// PRINCIPE :
// Calcule 1/sqrt(x) sans utiliser les instructions sqrt ou div qui sont lentes.
//
// FONCTIONNEMENT :
// 1. Réinterprète le float comme un entier (bit hack)
// 2. Applique une constante "magique" (0x5F3759DF) qui donne une approximation
// 3. Affine avec une itération de Newton-Raphson
//
// PRÉCISION : ~1% d'erreur, suffisant pour la physique temps réel
// PERFORMANCE : 3-4x plus rapide que 1.0 / x.sqrt() sur certains CPU
//
// NOTE SUR LE CODE UNSAFE :
// Le cast de f32 vers u32 via transmute est un "type punning" qui réinterprète
// les bits sans conversion. C'est unsafe car Rust ne peut pas vérifier que
// c'est intentionnel et sûr. Ici c'est parfaitement défini et voulu.
//
// =============================================================================

#[inline]
pub fn fast_inv_sqrt(x: f32) -> f32 {
    // Demi-valeur pour l'itération Newton-Raphson
    let x2 = x * 0.5;

    // Réinterprète les bits du float comme un entier 32 bits
    // C'est la partie "magique" de l'algorithme
    let i = unsafe { std::mem::transmute::<f32, u32>(x) };

    // La constante magique 0x5F3759DF
    // Cette valeur a été trouvée empiriquement et donne une excellente
    // approximation initiale pour l'inverse de la racine carrée
    let i = 0x5F3759DF - (i >> 1);

    // Reconvertit l'entier en float
    let y = unsafe { std::mem::transmute::<u32, f32>(i) };

    // Une itération de Newton-Raphson pour améliorer la précision
    // La formule y = y * (1.5 - x/2 * y²) converge vers 1/sqrt(x)
    y * (1.5 - x2 * y * y)
}

// =============================================================================
// Fonctions utilitaires
// =============================================================================

/// Crée un Vec2 - fonction helper pour une syntaxe concise
#[inline]
pub fn vec2(x: f32, y: f32) -> Vec2 {
    Vec2::new(x, y)
}

/// Crée un Vec3 - fonction helper pour définir des cercles (x, y, rayon)
#[inline]
pub fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

// =============================================================================
// Tests unitaires
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec2_length() {
        // Triangle 3-4-5 classique
        let v = Vec2::new(3.0, 4.0);
        assert!((v.length() - 5.0).abs() < 0.0001);
    }

    #[test]
    fn test_vec2_normalize() {
        let v = Vec2::new(3.0, 4.0);
        let n = v.normalize();
        assert!((n.length() - 1.0).abs() < 0.0001);
    }

    #[test]
    fn test_fast_inv_sqrt() {
        // Test avec x = 4.0, 1/sqrt(4) = 0.5
        let result = fast_inv_sqrt(4.0);
        let expected = 0.5;
        // Tolérance de 1%
        assert!((result - expected).abs() < 0.01);
    }

    #[test]
    fn test_vec2_normalize_fast() {
        let v = Vec2::new(3.0, 4.0);
        let n = v.normalize_fast();
        // La version rapide a une précision d'environ 1%
        assert!((n.length() - 1.0).abs() < 0.02);
    }

    #[test]
    fn test_vec2_dot_product() {
        // Vecteurs perpendiculaires : dot product = 0
        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        assert!((a.dot(b)).abs() < 0.0001);

        // Vecteurs parallèles
        let c = Vec2::new(2.0, 0.0);
        assert!((a.dot(c) - 2.0).abs() < 0.0001);
    }
}
