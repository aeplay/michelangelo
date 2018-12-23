use crate::mesh::Mesh;
use std::hash::Hash;
use std::collections::{HashMap, VecDeque};

struct MeshQueue<K: Hash + Eq> {
    meshes: VecDeque<(K, Mesh)>,
    total_vertices: usize,
    max_vertices: usize,
    dirty: bool,
}

impl<K: Hash + Eq> MeshQueue<K> {
    pub fn new(max_vertices: usize) -> MeshQueue<K> {
        MeshQueue {
            meshes: VecDeque::new(),
            total_vertices: 0,
            max_vertices,
            dirty: false,
        }
    }

    pub fn push(
        &mut self,
        new_members: Vec<(K, Mesh)>,
        total_new_vertices: usize,
    ) -> (Vec<(K, Mesh)>, usize) {
        if total_new_vertices > self.max_vertices {
            panic!("New meshes too big for one queue");
        }

        let mut dropped = Vec::new();
        let mut total_dropped_vertices = 0;

        while self.total_vertices + total_new_vertices > self.max_vertices {
            let next_member_to_drop = self
                .meshes
                .pop_front()
                .expect("Should still have meshes left");
            self.total_vertices -= next_member_to_drop.1.vertices.len();
            total_dropped_vertices += next_member_to_drop.1.vertices.len();
            dropped.push(next_member_to_drop);
        }

        self.meshes.extend(new_members);
        self.total_vertices += total_new_vertices;
        self.dirty = true;

        (dropped, total_dropped_vertices)
    }

    pub fn remove(&mut self, key: &K) {
        let index = self
            .meshes
            .iter()
            .position(|(k, _)| k == key)
            .expect("Should contain key to be removed");
        let (_, old_mesh) = self.meshes.remove(index).unwrap();
        self.total_vertices -= old_mesh.vertices.len();
        self.dirty = true;
    }

    pub fn get_mesh_if_changed(&mut self) -> Option<Mesh> {
        if self.dirty {
            self.dirty = false;
            Some(self.meshes.iter().map(|(_, mesh)| mesh).sum())
        } else {
            None
        }
    }
}

pub struct MeshGrouper<K: Hash + Eq + Clone> {
    groups: Vec<MeshQueue<K>>,
    group_membership: HashMap<K, usize>,
    max_vertices_per_group: usize,
}

pub struct GroupChange {
    pub group_id: usize,
    pub new_group_mesh: Mesh,
}

impl<K: Hash + Eq + Clone> MeshGrouper<K> {
    pub fn new(max_vertices_per_group: usize) -> MeshGrouper<K> {
        MeshGrouper {
            groups: Vec::new(),
            group_membership: HashMap::new(),
            max_vertices_per_group,
        }
    }

    pub fn update<RemI: IntoIterator<Item = K>, AddI: IntoIterator<Item = (K, Mesh)>>(
        &mut self,
        to_remove: RemI,
        to_add: AddI,
    ) -> Vec<GroupChange> {
        for key_to_remove in to_remove {
            let group_idx = self.group_membership[&key_to_remove];
            self.groups[group_idx].remove(&key_to_remove);
        }

        for new_member in to_add {
            let mut current_group_idx = 0;
            let new_member_n_vertices = new_member.1.vertices.len();
            let mut to_push = (vec![new_member], new_member_n_vertices);

            while !to_push.0.is_empty() {
                // all members that are currently to push will fit in the current group!
                for (member_key, _) in &to_push.0 {
                    self.group_membership
                        .insert(member_key.clone(), current_group_idx);
                }

                let found_group = if let Some(group) = self.groups.get_mut(current_group_idx) {
                    to_push = group.push(to_push.0, to_push.1);
                    current_group_idx += 1;
                    true
                } else {
                    false
                };

                if !found_group {
                    let mut new_group = MeshQueue::new(self.max_vertices_per_group);
                    to_push = new_group.push(to_push.0, to_push.1);
                    self.groups.push(new_group);
                    // the rest should always fit in the last new group
                    assert!(to_push.0.is_empty());
                }
            }
        }

        self.groups
            .iter_mut()
            .enumerate()
            .filter_map(|(i, group)| {
                group.get_mesh_if_changed().map(|mesh| GroupChange {
                    group_id: i,
                    new_group_mesh: mesh,
                })
            })
            .collect()
    }
}