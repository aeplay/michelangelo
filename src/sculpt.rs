use crate::mesh::{Mesh, Vertex};
use descartes::{P2, N, LinePath, PrimitiveArea, Band};
use lyon_tessellation::math::point as lyon_point;
use lyon_tessellation::path::iterator::PathIter;
use lyon_tessellation::path::PathEvent;
use lyon_tessellation::{FillOptions, FillTessellator};
use std::rc::Rc;

pub struct SculptLine {
    path: LinePath,
    z: N,
}

impl SculptLine {
    pub fn new(path: LinePath, z: N) -> Self {
        SculptLine {
            path, z
        }
    }
}

#[derive(Clone)]
pub struct SpannedSurface {
    left_line: Rc<SculptLine>,
    right_line: Rc<SculptLine>,
}

impl SpannedSurface {
    pub fn new(left_line: Rc<SculptLine>, right_line: Rc<SculptLine>) -> SpannedSurface {
        SpannedSurface {
            left_line,
            right_line,
        }
    }
}

#[derive(Clone)]
pub struct FlatSurface {
    boundary: Rc<SculptLine>,
}

impl FlatSurface {
    pub fn from_primitive_area(area: PrimitiveArea, z: N) -> Self {
        let boundary = Rc::new(SculptLine {
            path: area.boundary.path().clone(),
            z,
        });
        FlatSurface { boundary }
    }

    pub fn from_band(path: LinePath, width_left: N, width_right: N, z: N) -> Self {
        let boundary = Rc::new(SculptLine {
            path: Band::new_asymmetric(path, width_left, width_right).outline().0,
            z
        });
        FlatSurface { boundary }
    }

    pub fn extrude(&self, up: N, out: N) -> (SpannedSurface, FlatSurface) {
        let upper_boundary = Rc::new(SculptLine {
            path: if out == 0.0 {
                self.boundary.path.clone()
            } else {
                let shifted = self.boundary.path.shift_orthogonally(out);
                if let Some(shifted_ok) = shifted {
                    shifted_ok
                } else {
                    panic!(
                        "Couldn't shift {:?} to {:?}",
                        self.boundary.path,
                        self.boundary.path.shift_orthogonally_vectors().collect::<Vec<_>>()
                    );
                }
            },
            z: self.boundary.z + up,
        });
        let spanned_surface = SpannedSurface::new(self.boundary.clone(), upper_boundary.clone());
        let upper_surface = FlatSurface {
            boundary: upper_boundary,
        };
        (spanned_surface, upper_surface)
    }
}

pub enum Surface {
    Spanned(SpannedSurface),
    Flat(FlatSurface),
}

impl Into<Surface> for SpannedSurface {
    fn into(self) -> Surface {
        Surface::Spanned(self)
    }
}

impl Into<Surface> for FlatSurface {
    fn into(self) -> Surface {
        Surface::Flat(self)
    }
}

pub struct Sculpture(Vec<Surface>);

impl Sculpture {
    pub fn new(surfaces: Vec<Surface>) -> Self {
        Sculpture(surfaces)
    }

    pub fn push(&mut self, surface: Surface) {
        self.0.push(surface);
    }

    pub fn to_mesh(&self) -> Mesh {
        let mut mesh = Mesh::empty();

        for surface in self.0.iter() {
            match surface {
                Surface::Spanned(spanned_surface) => {
                    fn to_vertex(point: &P2, z: N) -> Vertex {
                        Vertex {
                            position: [point.x, point.y, z],
                        }
                    }

                    let left_points = &spanned_surface.left_line.path.points;
                    let right_points = &spanned_surface.right_line.path.points;

                    let vertices = left_points
                        .iter()
                        .map(|p| to_vertex(p, spanned_surface.left_line.z))
                        .chain(
                            right_points
                                .iter()
                                .map(|p| to_vertex(p, spanned_surface.right_line.z)),
                        ).collect::<Vec<_>>();

                    let left_len = left_points.len();

                    let indices = (0..(left_len - 1))
                        .flat_map(|left_i| {
                            let left_i = left_i as u16;
                            let right_i = left_i + left_len as u16;

                            vec![
                                left_i,
                                right_i.min(vertices.len() as u16 - 1),
                                left_i + 1,
                                left_i + 1,
                                right_i.min(vertices.len() as u16 - 1),
                                (right_i + 1).min(vertices.len() as u16 - 1),
                            ]
                        }).collect();

                    mesh += Mesh::new(vertices, indices);
                }
                Surface::Flat(flat_surface) => {
                    let first_point = flat_surface.boundary.path.points[0];
                    let path_iterator = PathIter::new(
                        Some(PathEvent::MoveTo(lyon_point(first_point.x, first_point.y)))
                            .into_iter()
                            .chain(
                                flat_surface.boundary.path.points[1..]
                                    .iter()
                                    .map(|point| PathEvent::LineTo(lyon_point(point.x, point.y))),
                            ),
                    );

                    let mut tesselator = FillTessellator::new();
                    let mut output = Mesh::empty();

                    tesselator
                        .tessellate_path(path_iterator, &FillOptions::default(), &mut output)
                        .unwrap();

                    for vertex in output.vertices.iter_mut() {
                        vertex.position[2] = flat_surface.boundary.z;
                    }

                    mesh += output;
                }
            }
        }

        mesh
    }
}
