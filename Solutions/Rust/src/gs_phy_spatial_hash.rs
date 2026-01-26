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
// gs_phy_spatial_hash.rs - Spatial partitioning for O(n) collision detection
// =============================================================================
//
// PROBLÈME RÉSOLU :
// -----------------
// Détecter les collisions entre N objets nécessite normalement de comparer
// chaque paire, soit O(n²) comparaisons. Avec 1000 objets = 1 million de tests !
//
// SOLUTION : SPATIAL HASH (Grille de hachage spatial)
// ---------------------------------------------------
// On divise l'espace en cellules de taille fixe. Chaque objet est inséré dans
// les cellules qu'il chevauche. Pour trouver les collisions potentielles d'un
// objet, on ne regarde que les cellules voisines.
//
// Complexité : O(n) en moyenne au lieu de O(n²)
//
// STRUCTURE DE LA GRILLE :
// ------------------------
//
//   +---+---+---+---+---+
//   |   | ● |   |   |   |  Chaque cellule contient une liste
//   +---+---+---+---+---+  de pointeurs vers les objets présents
//   |   |   | ● | ● |   |
//   +---+---+---+---+---+  Un objet peut être dans plusieurs cellules
//   | ● |   |   |   | ● |  s'il chevauche la frontière
//   +---+---+---+---+---+
//
// =============================================================================

// =============================================================================
// Constantes de configuration
// =============================================================================

/// Capacité initiale par cellule (nombre d'objets avant réallocation)
const INITIAL_CELL_CAPACITY: usize = 16;

/// Capacité maximale par cellule (évite les réallocations infinies)
const MAX_CELL_CAPACITY: usize = 64;

/// Capacité du buffer de résultats de query
const MAX_QUERY_RESULTS: usize = 256;

// =============================================================================
// SpatialHash - Structure principale
// =============================================================================

pub struct SpatialHash {
    /// Taille d'une cellule (en unités du monde)
    cell_size: f32,

    /// Inverse de cell_size (pour éviter les divisions)
    inv_cell_size: f32,

    /// Largeur de la grille en nombre de cellules
    grid_width: usize,

    /// Hauteur de la grille en nombre de cellules
    grid_height: usize,

    /// Décalage X (pour gérer les coordonnées négatives)
    offset_x: f32,

    /// Décalage Y
    offset_y: f32,

    /// Grille : tableau 2D aplati de cellules
    /// Chaque cellule contient un tableau de usize (tags encodés)
    cells: Vec<Vec<usize>>,

    /// Compteur d'objets par cellule
    cell_counts: Vec<usize>,

    /// Buffer de résultats pour les queries (pré-alloué)
    query_result: Vec<usize>,

    /// Nombre de résultats dans le buffer
    query_count: usize,
}

impl SpatialHash {
    /// Crée un nouveau spatial hash
    pub fn new() -> Self {
        Self {
            cell_size: 32.0,
            inv_cell_size: 1.0 / 32.0,
            grid_width: 0,
            grid_height: 0,
            offset_x: 0.0,
            offset_y: 0.0,
            cells: Vec::new(),
            cell_counts: Vec::new(),
            query_result: vec![0; MAX_QUERY_RESULTS],
            query_count: 0,
        }
    }

    /// Configure la grille pour un monde de dimensions données
    ///
    /// # Arguments
    /// * `world_width` - Largeur du monde
    /// * `world_height` - Hauteur du monde
    /// * `cell_size` - Taille des cellules (idéalement ~2x le rayon max des objets)
    pub fn setup(&mut self, world_width: f32, world_height: f32, cell_size: f32) {
        // Assure une taille minimale
        self.cell_size = cell_size.max(1.0);
        self.inv_cell_size = 1.0 / self.cell_size;

        // Calcule les dimensions de la grille
        self.grid_width = (world_width * self.inv_cell_size) as usize + 1;
        self.grid_height = (world_height * self.inv_cell_size) as usize + 1;
        self.offset_x = 0.0;
        self.offset_y = 0.0;

        let total_cells = self.grid_width * self.grid_height;

        // Alloue la grille avec capacité fixe par cellule
        self.cells = vec![Vec::with_capacity(MAX_CELL_CAPACITY); total_cells];
        self.cell_counts = vec![0; total_cells];
    }

    /// Vide toutes les cellules (sans désallouer la mémoire)
    pub fn clear(&mut self) {
        for cell in &mut self.cells {
            cell.clear();
        }
        for count in &mut self.cell_counts {
            *count = 0;
        }
    }

    /// Calcule l'index de cellule pour une position donnée
    ///
    /// # Returns
    /// `Some((cell_x, cell_y))` si dans les bornes, `None` sinon
    #[inline]
    fn get_cell_coords(&self, x: f32, y: f32) -> (i32, i32) {
        let cell_x = ((x - self.offset_x) * self.inv_cell_size) as i32;
        let cell_y = ((y - self.offset_y) * self.inv_cell_size) as i32;
        (cell_x, cell_y)
    }

    /// Convertit des coordonnées de cellule en index linéaire
    #[inline]
    fn cell_index(&self, cx: i32, cy: i32) -> Option<usize> {
        if cx >= 0 && (cx as usize) < self.grid_width
            && cy >= 0 && (cy as usize) < self.grid_height
        {
            Some(cy as usize * self.grid_width + cx as usize)
        } else {
            None
        }
    }

    /// Insère un objet dans la grille
    ///
    /// L'objet est inséré dans toutes les cellules qu'il chevauche.
    ///
    /// # Arguments
    /// * `tag` - Tag encodé (type + index)
    /// * `x`, `y` - Position du centre de l'objet
    /// * `radius` - Rayon (ou demi-diagonale pour les rectangles)
    pub fn insert(&mut self, tag: usize, x: f32, y: f32, radius: f32) {
        // Calcule les cellules couvertes par l'objet
        let (min_cx, min_cy) = self.get_cell_coords(x - radius, y - radius);
        let (max_cx, max_cy) = self.get_cell_coords(x + radius, y + radius);

        // Clamp aux bornes de la grille
        let min_cx = min_cx.max(0) as usize;
        let min_cy = min_cy.max(0) as usize;
        let max_cx = (max_cx as usize).min(self.grid_width.saturating_sub(1));
        let max_cy = (max_cy as usize).min(self.grid_height.saturating_sub(1));

        // Insère dans toutes les cellules couvertes
        for cy in min_cy..=max_cy {
            for cx in min_cx..=max_cx {
                let cell_idx = cy * self.grid_width + cx;

                // Vérifie qu'on ne dépasse pas la capacité max
                if self.cell_counts[cell_idx] < MAX_CELL_CAPACITY {
                    self.cells[cell_idx].push(tag);
                    self.cell_counts[cell_idx] += 1;
                }
            }
        }
    }

    /// Recherche les objets potentiellement en collision
    ///
    /// Retourne le nombre de résultats trouvés.
    /// Les résultats sont accessibles via `get_query_result()`.
    ///
    /// # Arguments
    /// * `self_tag` - Tag de l'objet qui fait la query (pour l'exclure)
    /// * `x`, `y` - Position de l'objet
    /// * `radius` - Rayon de recherche
    ///
    /// # Optimisation importante
    /// Pour éviter les doublons et les auto-collisions, on ne retourne que
    /// les objets dont le tag est SUPÉRIEUR à self_tag. Cela garantit que
    /// chaque paire n'est testée qu'une seule fois.
    pub fn query(&mut self, self_tag: usize, x: f32, y: f32, radius: f32) -> usize {
        self.query_count = 0;

        // Calcule les cellules à interroger
        let (min_cx, min_cy) = self.get_cell_coords(x - radius, y - radius);
        let (max_cx, max_cy) = self.get_cell_coords(x + radius, y + radius);

        // Clamp aux bornes
        let min_cx = min_cx.max(0) as usize;
        let min_cy = min_cy.max(0) as usize;
        let max_cx = (max_cx as usize).min(self.grid_width.saturating_sub(1));
        let max_cy = (max_cy as usize).min(self.grid_height.saturating_sub(1));

        // Parcourt les cellules
        for cy in min_cy..=max_cy {
            for cx in min_cx..=max_cx {
                let cell_idx = cy * self.grid_width + cx;

                // Parcourt les objets de la cellule
                for &tag in &self.cells[cell_idx] {
                    // Filtre : ne garde que les tags > self_tag
                    // Cela évite :
                    // 1. L'auto-collision (tag == self_tag)
                    // 2. Les doublons (chaque paire testée une seule fois)
                    if tag > self_tag {
                        if self.query_count < MAX_QUERY_RESULTS {
                            self.query_result[self.query_count] = tag;
                            self.query_count += 1;
                        }
                    }
                }
            }
        }

        self.query_count
    }

    /// Récupère un résultat de query par son index
    #[inline]
    pub fn get_query_result(&self, index: usize) -> usize {
        self.query_result[index]
    }

    /// Getter pour la taille des cellules
    #[inline]
    pub fn cell_size(&self) -> f32 {
        self.cell_size
    }
}

impl Default for SpatialHash {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// BoxGrid - Grille spécialisée pour les boîtes statiques
// =============================================================================
//
// Les boîtes statiques (murs, obstacles) ne bougent jamais.
// On utilise une grille séparée qui n'est reconstruite que quand des boîtes
// sont ajoutées ou supprimées (flag "dirty").
//
// =============================================================================

use crate::gs_phy_types::StaticBox;

/// Nombre maximum de boîtes par cellule
const BOX_GRID_MAX_PER_CELL: usize = 8;

/// Cellule de la grille de boîtes statiques
#[derive(Clone)]
pub struct BoxGridCell {
    /// Indices des boîtes dans cette cellule
    pub box_indices: [usize; BOX_GRID_MAX_PER_CELL],
    /// Nombre de boîtes dans cette cellule
    pub count: usize,
}

impl Default for BoxGridCell {
    fn default() -> Self {
        Self {
            box_indices: [0; BOX_GRID_MAX_PER_CELL],
            count: 0,
        }
    }
}

/// Grille pour les boîtes statiques
pub struct BoxGrid {
    /// Cellules de la grille
    pub cells: Vec<BoxGridCell>,

    /// Largeur de la grille
    pub width: usize,

    /// Hauteur de la grille
    pub height: usize,

    /// Taille des cellules
    pub cell_size: f32,

    /// Inverse de cell_size
    pub inv_cell_size: f32,

    /// Flag indiquant si la grille doit être reconstruite
    pub dirty: bool,
}

impl BoxGrid {
    pub fn new() -> Self {
        Self {
            cells: Vec::new(),
            width: 0,
            height: 0,
            cell_size: 32.0,
            inv_cell_size: 1.0 / 32.0,
            dirty: true,
        }
    }

    /// Reconstruit la grille à partir des boîtes
    pub fn rebuild(&mut self, world_width: f32, world_height: f32, boxes: &[StaticBox]) {
        self.width = (world_width * self.inv_cell_size) as usize + 1;
        self.height = (world_height * self.inv_cell_size) as usize + 1;
        let total_cells = self.width * self.height;

        // Réinitialise les cellules
        self.cells.clear();
        self.cells.resize(total_cells, BoxGridCell::default());

        // Insère chaque boîte dans les cellules qu'elle chevauche
        for (box_idx, b) in boxes.iter().enumerate() {
            let min_cx = ((b.min_x * self.inv_cell_size) as i32).max(0) as usize;
            let min_cy = ((b.min_y * self.inv_cell_size) as i32).max(0) as usize;
            let max_cx = ((b.max_x * self.inv_cell_size) as usize).min(self.width.saturating_sub(1));
            let max_cy = ((b.max_y * self.inv_cell_size) as usize).min(self.height.saturating_sub(1));

            for cy in min_cy..=max_cy {
                for cx in min_cx..=max_cx {
                    let cell_idx = cy * self.width + cx;
                    let cell = &mut self.cells[cell_idx];

                    if cell.count < BOX_GRID_MAX_PER_CELL {
                        cell.box_indices[cell.count] = box_idx;
                        cell.count += 1;
                    }
                }
            }
        }

        self.dirty = false;
    }

    /// Récupère la cellule pour une position donnée
    pub fn get_cell(&self, x: f32, y: f32) -> Option<&BoxGridCell> {
        let cx = (x * self.inv_cell_size) as i32;
        let cy = (y * self.inv_cell_size) as i32;

        if cx >= 0 && (cx as usize) < self.width
            && cy >= 0 && (cy as usize) < self.height
        {
            let idx = cy as usize * self.width + cx as usize;
            Some(&self.cells[idx])
        } else {
            None
        }
    }
}

impl Default for BoxGrid {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Tests unitaires
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spatial_hash_setup() {
        let mut hash = SpatialHash::new();
        hash.setup(800.0, 600.0, 32.0);

        assert_eq!(hash.grid_width, 26); // 800/32 + 1
        assert_eq!(hash.grid_height, 19); // 600/32 + 1
    }

    #[test]
    fn test_spatial_hash_insert_query() {
        let mut hash = SpatialHash::new();
        hash.setup(100.0, 100.0, 10.0);

        // Insère deux objets proches
        hash.insert(1, 15.0, 15.0, 5.0);
        hash.insert(2, 18.0, 15.0, 5.0);

        // Query depuis le premier objet
        let count = hash.query(1, 15.0, 15.0, 5.0);

        // Devrait trouver l'objet 2 (tag > 1)
        assert!(count >= 1);
    }

    #[test]
    fn test_box_grid() {
        let mut grid = BoxGrid::new();
        let boxes = vec![
            StaticBox::new(50.0, 50.0, 20.0, 20.0, 0.3),
        ];

        grid.rebuild(100.0, 100.0, &boxes);
        assert!(!grid.dirty);

        // La boîte devrait être dans la cellule autour de (50, 50)
        let cell = grid.get_cell(50.0, 50.0);
        assert!(cell.is_some());
        assert!(cell.unwrap().count > 0);
    }
}
