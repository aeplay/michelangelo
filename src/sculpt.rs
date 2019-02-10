use crate::mesh::{Mesh, Vertex};
use descartes::{P2, N, LinePath, PrimitiveArea, Band};
use lyon_tessellation::math::point as lyon_point;
use lyon_tessellation::path::iterator::PathIter;
use lyon_tessellation::path::PathEvent;
use lyon_tessellation::{FillOptions, FillTessellator};
use std::rc::Rc;

pub struct SculptLine {
    pub path: LinePath,
    pub z: N,
}

impl SculptLine {
    pub fn new(path: LinePath, z: N) -> Self {
        SculptLine {
            path, z
        }
    }

    pub fn extrude(line: &Rc<Self>, up: N, out: N) -> Option<(SpannedSurface, Rc<SculptLine>)> {
        let upper_line = Rc::new(SculptLine {
            path: if out == 0.0 {
                line.path.clone()
            } else {
                line.path.shift_orthogonally(out)?
            },
            z: line.z + up,
        });
        let spanned_surface = SpannedSurface::new(line.clone(), upper_line.clone());
        Some((spanned_surface, upper_line))
    }
}

#[derive(Clone)]
pub struct SpannedSurface {
    pub left_line: Rc<SculptLine>,
    pub right_line: Rc<SculptLine>,
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
    pub boundary: Rc<SculptLine>,
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

    pub fn extrude(&self, up: N, out: N) -> Option<(SpannedSurface, FlatSurface)> {
        let (spanned_surface, upper_boundary) = SculptLine::extrude(&self.boundary, up, out)?;
        let upper_surface = FlatSurface {
            boundary: upper_boundary.clone(),
        };
        Some((spanned_surface, upper_surface))
    }
}

#[derive(Clone)]
pub struct SkeletonSpine {
    center: Rc<SculptLine>,
    width: N,
    boundary: Rc<SculptLine>,
    left_length: N,
    front_length: N,
    right_length: N,
    back_length: N
}

impl SkeletonSpine {
    pub fn new(center: Rc<SculptLine>, width: N) -> Option<SkeletonSpine> {
        let left = center.path.shift_orthogonally(-width / 2.0)?;
        let right = center.path.shift_orthogonally(width / 2.0)?;
        let back = LinePath::new(vec![right.points[0], left.points[0]].into())?;
        let front = LinePath::new(
                vec![
                    *left.points.last().unwrap(),
                    *right.points.last().unwrap(),
                ]
                .into(),
            )?;
        let boundary = Rc::new(SculptLine::new(left.concat(&front).ok()?.concat(&right.reverse()).ok()?.concat(&back).ok()?, center.z));
        Some(SkeletonSpine {
            width,
            center,
            boundary,
            left_length: left.length(),
            front_length: front.length(),
            right_length: right.length(),
            back_length: back.length()
        })
    }

    pub fn extrude(&self, up: N, widen_by: N, extend_by: N) -> Option<(SpannedSurface, Self)> {
        let new_center = if extend_by == 0.0 {
            if up == 0.0 {
                self.center.clone()
            } else {
                Rc::new(SculptLine::new(self.center.path.clone(), self.center.z + up))
            }
        } else {
            let p = &self.center.path;
                Rc::new(SculptLine::new(
                    p.with_new_start_and_end(p.start() - extend_by * p.start_direction(), p.end() + extend_by * p.end_direction())?,
                    self.center.z + up
                ))
        };

        let new_spine = SkeletonSpine::new(new_center, self.width + widen_by)?;
        let surface = SpannedSurface::new(self.boundary.clone(), new_spine.boundary.clone());
        Some((surface, new_spine))
    }

    pub fn roof(&self, height: N, gable_depth_front: N, gable_depth_back: N) -> (RoofSurface, GableSurface) {
        (RoofSurface{spine: self.clone(), height, gable_depth_front, gable_depth_back}, GableSurface{spine: self.clone(), height, gable_depth_front, gable_depth_back})
    }
}

pub struct RoofSurface {
    spine: SkeletonSpine,
    height: N,
    gable_depth_front: N,
    gable_depth_back: N
}

pub struct GableSurface {
    spine: SkeletonSpine,
    height: N,
    gable_depth_front: N,
    gable_depth_back: N
}

pub enum Surface {
    Spanned(SpannedSurface),
    Flat(FlatSurface),
    Roof(RoofSurface),
    Gable(GableSurface)
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

impl Into<Surface> for RoofSurface {
    fn into(self) -> Surface {
        Surface::Roof(self)
    }
}

impl Into<Surface> for GableSurface {
    fn into(self) -> Surface {
        Surface::Gable(self)
    }
}

pub struct Sculpture(Vec<Surface>);

fn to_vertex(point: &P2, z: N) -> Vertex {
    Vertex {
        position: [point.x, point.y, z],
    }
}

fn strip_indices(left_start_i: usize, left_len: usize, right_start_i: usize, right_len: usize, reverse_right: bool) -> Vec<u16> {
    if reverse_right {
        (0..(left_len - 1))
            .flat_map(|i| {
                let left_i = (i + left_start_i) as u16;
                let right_i = (right_start_i + right_len - 1 -i) as u16;

                vec![
                    left_i,
                    right_i.max(right_start_i as u16),
                    left_i + 1,
                    left_i + 1,
                    right_i.max(right_start_i as u16),
                    (right_i + 1).max(right_start_i as u16),
                ]
            }).collect()
    } else {
        (0..(left_len - 1))
            .flat_map(|i| {
                let left_i = (i + left_start_i) as u16;
                let right_i = (i + right_start_i) as u16;

                vec![
                    left_i,
                    right_i.min((right_start_i + right_len) as u16 - 1),
                    left_i + 1,
                    left_i + 1,
                    right_i.min((right_start_i + right_len) as u16 - 1),
                    (right_i + 1).min((right_start_i + right_len) as u16 - 1),
                ]
            }).collect()
    }
}

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

                    // let left_len = left_points.len();

                    // let indices = (0..(left_len - 1))
                    //     .flat_map(|left_i| {
                    //         let left_i = left_i as u16;
                    //         let right_i = left_i + left_len as u16;

                    //         vec![
                    //             left_i,
                    //             right_i.min(vertices.len() as u16 - 1),
                    //             left_i + 1,
                    //             left_i + 1,
                    //             right_i.min(vertices.len() as u16 - 1),
                    //             (right_i + 1).min(vertices.len() as u16 - 1),
                    //         ]
                    //     }).collect();
                    let indices = strip_indices(0, left_points.len(), left_points.len(), right_points.len(), false);

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
                },
                Surface::Roof(roof_surface) => {
                    let center_path = &roof_surface.spine.center.path;
                    let ridge_points = Some(center_path.along(roof_surface.gable_depth_back)).into_iter().chain(center_path.points[1..=(center_path.points.len() - 2)].iter().cloned()).chain(Some(center_path.along(center_path.length() - roof_surface.gable_depth_front))).collect::<Vec<_>>();
                    let boundary_points = &roof_surface.spine.boundary.path.points;

                    let vertices = boundary_points.iter().map(|p| to_vertex(p, roof_surface.spine.center.z)).chain(ridge_points.iter().map(|p| to_vertex(p, roof_surface.spine.center.z + roof_surface.height))).collect();
                    let indices = strip_indices(0, boundary_points.len() / 2, boundary_points.len(), ridge_points.len(), false).into_iter().chain(
                        strip_indices(boundary_points.len() / 2, boundary_points.len() / 2, boundary_points.len(), ridge_points.len(), true)
                    ).collect();

                    mesh += Mesh::new(vertices, indices);
                }
                Surface::Gable(gable_surface) => {
                    let center_path = &gable_surface.spine.center.path;
                    let center_back = center_path.along(gable_surface.gable_depth_back);
                    let center_front = center_path.along(center_path.length() - gable_surface.gable_depth_front);
                    let boundary_points = &gable_surface.spine.boundary.path.points;

                    let low_z = gable_surface.spine.center.z;
                    let high_z = low_z + gable_surface.height;

                    let vertices = vec![
                        to_vertex(&boundary_points[0], low_z), to_vertex(&boundary_points[boundary_points.len() - 1], low_z), to_vertex(&center_back, high_z),
                        to_vertex(&boundary_points[boundary_points.len() / 2 - 1], low_z), to_vertex(&boundary_points[boundary_points.len() / 2], low_z), to_vertex(&center_front, high_z)
                    ];
                    let indices = vec![0, 1, 2, 3, 4, 5];

                    mesh += Mesh::new(vertices, indices);
                }
            }
        }

        mesh
    }
}
