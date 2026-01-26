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
// gs_phy2d_demo.rs - Interactive physics engine demonstration
// =============================================================================
//
// Cette démo reproduit l'application GSPhy2DDemo de Delphi.
//
// CONTRÔLES :
// -----------
// [1] ou [B] : Ajouter 50 balles (particules)
// [2] ou [A] : Ajouter 50 AABB dynamiques
// [3] ou [S] : Ajouter 50 boîtes statiques (murs)
// [4] ou [R] : Ajouter 10 corps rigides (rectangles)
// [C]        : Effacer tout
// [+]/[-]    : Augmenter/diminuer les itérations de collision
// [D]        : Augmenter le damping
// [Shift+D]  : Diminuer le damping
// [E]        : Augmenter la restitution (élasticité)
// [Shift+E]  : Diminuer la restitution
// [Escape]   : Quitter
//
// =============================================================================

// On utilise la bibliothèque qu'on vient de créer
use gs_phy2d::{PhyWorld, velocity_to_color, color_to_rgba, colors};

// macroquad pour le rendu et les entrées
use macroquad::prelude::*;

// rand pour les positions aléatoires
use ::rand::Rng;

// =============================================================================
// Configuration de la fenêtre
// =============================================================================

fn window_conf() -> Conf {
    Conf {
        window_title: "GS.Phy2D Demo - Portage Rust".to_owned(),
        window_width: 1024,
        window_height: 768,
        window_resizable: true,
        ..Default::default()
    }
}

// =============================================================================
// Constantes
// =============================================================================

const BALL_RADIUS_MIN: f32 = 3.0;
const BALL_RADIUS_MAX: f32 = 10.0;
const AABB_MIN_SIZE: f32 = 10.0;
const AABB_MAX_SIZE: f32 = 30.0;
const BOX_MIN_SIZE: f32 = 15.0;
const BOX_MAX_SIZE: f32 = 40.0;
const RECT_MIN_SIZE: f32 = 30.0;
const RECT_MAX_SIZE: f32 = 60.0;
const MAX_SPEED: f32 = 15.0;  // Pour le gradient de couleur

// =============================================================================
// Fonctions de spawn
// =============================================================================

/// Ajoute des balles (particules) aléatoirement
fn spawn_balls(world: &mut PhyWorld, count: usize) {
    let mut rng = ::rand::thread_rng();

    let spawn_left = BALL_RADIUS_MAX + 10.0;
    let spawn_right = world.world_width() - BALL_RADIUS_MAX - 10.0;
    let spawn_top = BALL_RADIUS_MAX + 10.0;
    let spawn_bottom = world.world_height() / 3.0;

    for _ in 0..count {
        let x = rng.gen_range(spawn_left..spawn_right);
        let y = rng.gen_range(spawn_top..spawn_bottom);
        let radius = rng.gen_range(BALL_RADIUS_MIN..BALL_RADIUS_MAX);

        world.add_particle(x, y, radius, false, 1.0, 0.5);
    }
}

/// Ajoute des AABB dynamiques aléatoirement
fn spawn_aabbs(world: &mut PhyWorld, count: usize) {
    let mut rng = ::rand::thread_rng();

    let spawn_left = AABB_MAX_SIZE + 10.0;
    let spawn_right = world.world_width() - AABB_MAX_SIZE - 10.0;
    let spawn_top = AABB_MAX_SIZE + 10.0;
    let spawn_bottom = world.world_height() / 3.0;

    for _ in 0..count {
        let x = rng.gen_range(spawn_left..spawn_right);
        let y = rng.gen_range(spawn_top..spawn_bottom);
        let w = rng.gen_range(AABB_MIN_SIZE..AABB_MAX_SIZE);
        let h = rng.gen_range(AABB_MIN_SIZE..AABB_MAX_SIZE);

        world.add_aabb(x, y, w, h, false, 1.0, 0.5);
    }
}

/// Ajoute des boîtes statiques (murs) aléatoirement
fn spawn_boxes(world: &mut PhyWorld, count: usize) {
    let mut rng = ::rand::thread_rng();

    let spawn_left = BOX_MAX_SIZE + 20.0;
    let spawn_right = world.world_width() - BOX_MAX_SIZE - 20.0;
    let spawn_top = world.world_height() / 3.0;
    let spawn_bottom = world.world_height() - BOX_MAX_SIZE - 20.0;

    for _ in 0..count {
        let x = rng.gen_range(spawn_left..spawn_right);
        let y = rng.gen_range(spawn_top..spawn_bottom);
        let w = rng.gen_range(BOX_MIN_SIZE..BOX_MAX_SIZE);
        let h = rng.gen_range(BOX_MIN_SIZE..BOX_MAX_SIZE);

        world.add_box(x, y, w, h, 0.3);
    }
}

/// Ajoute des corps rigides (rectangles composés de cercles)
fn spawn_rigid_bodies(world: &mut PhyWorld, count: usize) {
    let mut rng = ::rand::thread_rng();

    let spawn_left = RECT_MAX_SIZE + 20.0;
    let spawn_right = world.world_width() - RECT_MAX_SIZE - 20.0;
    let spawn_top = RECT_MAX_SIZE + 20.0;
    let spawn_bottom = world.world_height() / 3.0;

    for _ in 0..count {
        let x = rng.gen_range(spawn_left..spawn_right);
        let y = rng.gen_range(spawn_top..spawn_bottom);
        let w = rng.gen_range(RECT_MIN_SIZE..RECT_MAX_SIZE);
        let h = rng.gen_range(RECT_MIN_SIZE..RECT_MAX_SIZE);

        world.add_rect_body(x, y, w, h, 1.0, 0.3);
    }
}

// =============================================================================
// Fonction de rendu
// =============================================================================

/// Convertit une couleur PhyColor en Color macroquad
fn phy_to_mq(color: u32) -> Color {
    let (r, g, b, a) = color_to_rgba(color);
    Color::new(r, g, b, a)
}

/// Dessine tout le monde physique
fn render_world(world: &PhyWorld) {
    let w = world.world_width();
    let h = world.world_height();

    // Bordures du monde
    let border_color = phy_to_mq(colors::LIGHT_GRAY);
    draw_rectangle(0.0, 0.0, w, 2.0, border_color);
    draw_rectangle(0.0, h - 2.0, w, 2.0, border_color);
    draw_rectangle(0.0, 0.0, 2.0, h, border_color);
    draw_rectangle(w - 2.0, 0.0, 2.0, h, border_color);

    // Boîtes statiques
    let box_color = phy_to_mq(colors::GRAY);
    for i in 0..world.box_count() {
        let b = world.get_box(i);
        draw_rectangle(
            b.min_x,
            b.min_y,
            b.max_x - b.min_x,
            b.max_y - b.min_y,
            box_color,
        );
    }

    // Particules avec couleur de vélocité
    for i in 0..world.particle_count() {
        let pos_x = world.get_pos_x(i);
        let pos_y = world.get_pos_y(i);
        let old_pos_x = world.get_old_pos_x(i);
        let old_pos_y = world.get_old_pos_y(i);
        let radius = world.get_radius(i);

        let vel_x = pos_x - old_pos_x;
        let vel_y = pos_y - old_pos_y;
        let color = velocity_to_color(vel_x, vel_y, MAX_SPEED);

        draw_circle(pos_x, pos_y, radius, phy_to_mq(color));
    }

    // AABB dynamiques avec couleur de vélocité
    for i in 0..world.aabb_count() {
        let pos_x = world.get_aabb_pos_x(i);
        let pos_y = world.get_aabb_pos_y(i);
        let old_pos_x = world.get_aabb_old_pos_x(i);
        let old_pos_y = world.get_aabb_old_pos_y(i);
        let half_w = world.get_aabb_half_w(i);
        let half_h = world.get_aabb_half_h(i);

        let vel_x = pos_x - old_pos_x;
        let vel_y = pos_y - old_pos_y;
        let color = velocity_to_color(vel_x, vel_y, MAX_SPEED);

        draw_rectangle(
            pos_x - half_w,
            pos_y - half_h,
            half_w * 2.0,
            half_h * 2.0,
            phy_to_mq(color),
        );
    }

    // Contraintes (lignes rouges)
    let constraint_color = phy_to_mq(colors::RED);
    for i in 0..world.constraint_count() {
        let c = world.get_constraint(i);
        let x1 = world.get_pos_x(c.p1);
        let y1 = world.get_pos_y(c.p1);
        let x2 = world.get_pos_x(c.p2);
        let y2 = world.get_pos_y(c.p2);
        draw_line(x1, y1, x2, y2, 2.0, constraint_color);
    }
}

/// Dessine l'interface utilisateur
fn render_ui(world: &PhyWorld, fps: i32) {
    let text_color = YELLOW;
    let font_size = 18.0;
    let mut y = 20.0;
    let line_height = 22.0;

    // Titre et FPS
    draw_text(
        &format!("GS.Phy2D Demo | FPS: {}", fps),
        10.0, y, font_size, text_color
    );
    y += line_height;

    // Compteurs
    let total = world.particle_count() + world.aabb_count();
    draw_text(
        &format!(
            "Dynamiques: {} (Balles: {}, AABB: {})",
            total,
            world.particle_count(),
            world.aabb_count()
        ),
        10.0, y, font_size, text_color
    );
    y += line_height;

    draw_text(
        &format!(
            "Statiques: {} | Corps rigides: {}",
            world.box_count(),
            world.rigid_body_count()
        ),
        10.0, y, font_size, text_color
    );
    y += line_height;

    // Paramètres
    draw_text(
        &format!(
            "Iter: {} | Damping: {:.0}% | Restitution: {:.0}%",
            world.collision_iterations(),
            world.damping() * 100.0,
            world.restitution() * 100.0
        ),
        10.0, y, font_size, text_color
    );
    y += line_height;

    // Contrôles
    draw_text(
        "[1/B] Balles | [2/A] AABB | [3/S] Murs | [4/R] Corps rigides | [C] Clear",
        10.0, y, font_size, text_color
    );
    y += line_height;

    draw_text(
        "[+/-] Iter | [D/Shift+D] Damping | [E/Shift+E] Restitution | [Esc] Quit",
        10.0, y, font_size, text_color
    );
}

// =============================================================================
// Point d'entrée
// =============================================================================

#[macroquad::main(window_conf)]
async fn main() {
    // =========================================================================
    // INITIALISATION
    // =========================================================================

    let mut world = PhyWorld::new();
    world.set_gravity(0.0, 800.0);
    world.set_damping(0.99);
    world.set_collision_iterations(2);

    // Variables pour le calcul du FPS
    let mut frame_count = 0;
    let mut last_fps_update = get_time();
    let mut fps = 0;

    // =========================================================================
    // BOUCLE PRINCIPALE
    // =========================================================================

    loop {
        // =====================================================================
        // Mise à jour des dimensions du monde (si fenêtre redimensionnée)
        // =====================================================================
        let sw = screen_width();
        let sh = screen_height();
        if (world.world_width() - sw).abs() > 1.0 || (world.world_height() - sh).abs() > 1.0 {
            world.set_world_bounds(sw, sh);
        }

        // =====================================================================
        // GESTION DES ENTRÉES
        // =====================================================================

        // Ajouter des balles
        if is_key_pressed(KeyCode::Key1) || is_key_pressed(KeyCode::B) {
            spawn_balls(&mut world, 50);
        }

        // Ajouter des AABB
        if is_key_pressed(KeyCode::Key2) || is_key_pressed(KeyCode::A) {
            spawn_aabbs(&mut world, 50);
        }

        // Ajouter des boîtes statiques
        if is_key_pressed(KeyCode::Key3) || is_key_pressed(KeyCode::S) {
            spawn_boxes(&mut world, 50);
        }

        // Ajouter des corps rigides
        if is_key_pressed(KeyCode::Key4) || is_key_pressed(KeyCode::R) {
            spawn_rigid_bodies(&mut world, 10);
        }

        // Effacer
        if is_key_pressed(KeyCode::C) {
            world.clear();
        }

        // Itérations +/-
        if is_key_pressed(KeyCode::Equal) || is_key_pressed(KeyCode::KpAdd) {
            let iter = world.collision_iterations();
            world.set_collision_iterations(iter + 1);
        }
        if is_key_pressed(KeyCode::Minus) || is_key_pressed(KeyCode::KpSubtract) {
            let iter = world.collision_iterations();
            if iter > 1 {
                world.set_collision_iterations(iter - 1);
            }
        }

        // Damping
        if is_key_pressed(KeyCode::D) {
            let shift = is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift);
            let current = world.damping();
            if shift {
                world.set_damping((current - 0.05).max(0.0));
            } else {
                world.set_damping((current + 0.05).min(1.0));
            }
        }

        // Restitution
        if is_key_pressed(KeyCode::E) {
            let shift = is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift);
            let current = world.restitution();
            if shift {
                world.set_restitution((current - 0.1).max(0.0));
            } else {
                world.set_restitution((current + 0.1).min(1.0));
            }
        }

        // Quitter
        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        // =====================================================================
        // SIMULATION
        // =====================================================================

        world.step(1.0 / 60.0);

        // =====================================================================
        // CALCUL DU FPS
        // =====================================================================

        frame_count += 1;
        let now = get_time();
        if now - last_fps_update >= 0.5 {
            fps = (frame_count as f64 / (now - last_fps_update)) as i32;
            frame_count = 0;
            last_fps_update = now;
        }

        // =====================================================================
        // RENDU
        // =====================================================================

        clear_background(WHITE);
        render_world(&world);
        render_ui(&world, fps);

        // =====================================================================
        // FRAME SUIVANTE
        // =====================================================================

        next_frame().await;
    }
}
