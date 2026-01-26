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
// gs_phy_renderer.rs - Abstract rendering system and utilities
// =============================================================================
//
// Ce module fournit :
// - Des couleurs prédéfinies (format ARGB 32 bits)
// - Une fonction de conversion vélocité → couleur (gradient thermique)
// - Le rendu est fait directement avec macroquad dans la démo
//
// Note : En Delphi, il y avait une classe abstraite TPhyRenderer.
// En Rust, on n'a pas besoin de cette abstraction car macroquad est
// suffisamment simple. On garde juste les utilitaires.
//
// =============================================================================

// =============================================================================
// PhyColor - Couleur ARGB 32 bits
// =============================================================================
//
// Format : 0xAARRGGBB
// - AA : Alpha (transparence, FF = opaque)
// - RR : Rouge (00-FF)
// - GG : Vert (00-FF)
// - BB : Bleu (00-FF)
//
// =============================================================================

pub type PhyColor = u32;

/// Module de couleurs prédéfinies
pub mod colors {
    use super::PhyColor;

    pub const WHITE: PhyColor     = 0xFFFFFFFF;
    pub const BLACK: PhyColor     = 0xFF000000;
    pub const RED: PhyColor       = 0xFFFF0000;
    pub const GREEN: PhyColor     = 0xFF00FF00;
    pub const BLUE: PhyColor      = 0xFF0000FF;
    pub const YELLOW: PhyColor    = 0xFFFFFF00;
    pub const CYAN: PhyColor      = 0xFF00FFFF;
    pub const MAGENTA: PhyColor   = 0xFFFF00FF;
    pub const GRAY: PhyColor      = 0xFF808080;
    pub const LIGHT_GRAY: PhyColor = 0xFFD3D3D3;
    pub const DARK_GRAY: PhyColor = 0xFF404040;
}

// =============================================================================
// Conversion vélocité → couleur
// =============================================================================
//
// Crée un gradient de couleur basé sur la vitesse :
// - Bleu : lent (froid)
// - Cyan : un peu plus rapide
// - Vert : vitesse moyenne
// - Jaune : rapide
// - Rouge : très rapide (chaud)
//
// C'est une visualisation "thermique" intuitive de la vitesse.
//
// =============================================================================

/// Convertit une vélocité en couleur (gradient thermique)
///
/// # Arguments
/// * `vel_x`, `vel_y` - Composantes de la vélocité
/// * `max_speed` - Vitesse maximale pour normalisation
///
/// # Returns
/// Couleur ARGB
pub fn velocity_to_color(vel_x: f32, vel_y: f32, max_speed: f32) -> PhyColor {
    // Calcule la magnitude de la vitesse
    let speed = (vel_x * vel_x + vel_y * vel_y).sqrt();

    // Normalise entre 0 et 1
    let speed_norm = (speed / max_speed).min(1.0);

    // Gradient : Bleu → Cyan → Vert → Jaune → Rouge
    //            0.0    0.25    0.5    0.75    1.0

    let (r, g, b) = if speed_norm < 0.25 {
        // Bleu → Cyan : augmente le vert
        let t = speed_norm * 4.0;
        (0, (t * 255.0) as u8, 255)
    } else if speed_norm < 0.5 {
        // Cyan → Vert : diminue le bleu
        let t = (speed_norm - 0.25) * 4.0;
        (0, 255, ((1.0 - t) * 255.0) as u8)
    } else if speed_norm < 0.75 {
        // Vert → Jaune : augmente le rouge
        let t = (speed_norm - 0.5) * 4.0;
        ((t * 255.0) as u8, 255, 0)
    } else {
        // Jaune → Rouge : diminue le vert
        let t = (speed_norm - 0.75) * 4.0;
        (255, ((1.0 - t) * 255.0) as u8, 0)
    };

    // Combine en ARGB
    0xFF000000 | ((r as u32) << 16) | ((g as u32) << 8) | (b as u32)
}

/// Convertit une couleur PhyColor en Color de macroquad
#[cfg(feature = "macroquad")]
pub fn to_macroquad_color(color: PhyColor) -> macroquad::prelude::Color {
    let a = ((color >> 24) & 0xFF) as f32 / 255.0;
    let r = ((color >> 16) & 0xFF) as f32 / 255.0;
    let g = ((color >> 8) & 0xFF) as f32 / 255.0;
    let b = (color & 0xFF) as f32 / 255.0;
    macroquad::prelude::Color::new(r, g, b, a)
}

/// Convertit une couleur PhyColor en tuple (r, g, b, a) normalisé
pub fn color_to_rgba(color: PhyColor) -> (f32, f32, f32, f32) {
    let a = ((color >> 24) & 0xFF) as f32 / 255.0;
    let r = ((color >> 16) & 0xFF) as f32 / 255.0;
    let g = ((color >> 8) & 0xFF) as f32 / 255.0;
    let b = (color & 0xFF) as f32 / 255.0;
    (r, g, b, a)
}

// =============================================================================
// Tests unitaires
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_velocity_to_color_slow() {
        // Vitesse nulle = bleu
        let color = velocity_to_color(0.0, 0.0, 15.0);
        let b = (color & 0xFF) as u8;
        assert_eq!(b, 255); // Bleu max
    }

    #[test]
    fn test_velocity_to_color_fast() {
        // Vitesse max = rouge
        let color = velocity_to_color(15.0, 0.0, 15.0);
        let r = ((color >> 16) & 0xFF) as u8;
        assert_eq!(r, 255); // Rouge max
    }

    #[test]
    fn test_color_to_rgba() {
        let (r, g, b, a) = color_to_rgba(0xFFFF8040);
        assert!((a - 1.0).abs() < 0.01);
        assert!((r - 1.0).abs() < 0.01);
        assert!((g - 0.5).abs() < 0.1);
        assert!((b - 0.25).abs() < 0.1);
    }
}
