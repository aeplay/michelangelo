extern crate descartes;
extern crate compact;
#[macro_use]
extern crate compact_macros;
extern crate itertools;
extern crate lyon_tessellation;

mod mesh;
mod mesh_grouper;

pub use mesh::{Mesh, Vertex, Instance};
pub use mesh_grouper::{MeshGrouper, GroupChange};
