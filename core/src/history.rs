//! Bounded execution checkpoints. IDs survive clears so delayed requests cannot select a new checkpoint.
use crate::world::World;
use std::collections::VecDeque;

pub const HISTORY_HZ: u32 = 30;
pub const HISTORY_FRAMES: usize = 600;
pub const HISTORY_BYTES: usize = 64 * 1024 * 1024;

#[derive(Debug)]
struct Checkpoint {
    id: u32,
    bytes: usize,
    world: World,
}

#[derive(Debug, Default)]
pub struct History {
    frames: VecDeque<Checkpoint>,
    next: u32,
    bytes: usize,
}
impl History {
    pub fn clear(&mut self) {
        self.frames.clear();
        self.bytes = 0;
    }
    pub fn save(&mut self, world: &World) -> u32 {
        let cost = world.retained_bytes();
        if cost > HISTORY_BYTES {
            self.clear();
            return 0;
        }
        self.next = self.next.wrapping_add(1).max(1);
        while self.frames.len() >= HISTORY_FRAMES || self.bytes + cost > HISTORY_BYTES {
            if let Some(frame) = self.frames.pop_front() {
                self.bytes -= frame.bytes;
            }
        }
        self.frames.push_back(Checkpoint { id: self.next, bytes: cost, world: world.clone() });
        self.bytes += cost;
        self.next
    }
    #[must_use]
    pub fn oldest(&self) -> u32 {
        self.frames.front().map_or(0, |f| f.id)
    }
    /// Restore the complete world and discard the superseded future. Failure leaves both untouched.
    pub fn restore(&mut self, id: u32, world: &mut World) -> bool {
        let Some(index) = self.frames.iter().position(|f| f.id == id) else { return false };
        world.clone_from(&self.frames[index].world);
        while self.frames.len() > index + 1 {
            if let Some(frame) = self.frames.pop_back() {
                self.bytes -= frame.bytes;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mech::presets, model::Model};

    #[test]
    fn retention_obeys_both_budgets_and_expired_restore_is_atomic() {
        let mut world = World::new(Model::from_desc(&presets::arm2()));
        let mut history = History::default();
        let first = history.save(&world);
        for _ in 0..HISTORY_FRAMES + 2 {
            history.save(&world);
        }
        assert!(history.frames.len() <= HISTORY_FRAMES);
        assert!(history.bytes <= HISTORY_BYTES);
        assert_ne!(history.oldest(), first);
        world.step(1);
        let t = world.primary().t;
        let count = history.frames.len();
        assert!(!history.restore(first, &mut world));
        assert_eq!(world.primary().t, t);
        assert_eq!(history.frames.len(), count);
        let last = history.frames.back().unwrap().id;
        history.clear();
        assert_eq!(history.oldest(), 0);
        assert!(history.frames.is_empty());
        assert_ne!(history.save(&world), last);
        let mut world = World::new(Model::from_desc(&presets::arm7()));
        history.clear();
        for slot in 1..crate::layout::SLOTS {
            world.set_controller(slot, Some(crate::control::Kind::Passive));
        }
        for _ in 0..HISTORY_FRAMES + 2 {
            history.save(&world);
        }
        assert!(history.frames.len() < HISTORY_FRAMES, "larger worlds must hit the byte budget first");
        assert!(history.bytes <= HISTORY_BYTES);
        history.next = u32::MAX;
        assert_eq!(history.save(&world), 1);
    }
}
