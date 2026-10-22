//! Bounded execution checkpoints. IDs survive clears so delayed requests cannot select a new checkpoint.
use crate::world::World;
use std::collections::VecDeque;

pub const HISTORY_HZ: u32 = 30;
pub const HISTORY_FRAMES: usize = 600;
pub const HISTORY_BYTES: usize = 64 * 1024 * 1024;

#[derive(Debug, Default)]
pub struct History {
    frames: VecDeque<(u32, usize, World)>,
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
            if let Some((_, bytes, _)) = self.frames.pop_front() {
                self.bytes -= bytes;
            }
        }
        self.frames.push_back((self.next, cost, world.clone()));
        self.bytes += cost;
        self.next
    }
    #[must_use]
    pub fn oldest(&self) -> u32 {
        self.frames.front().map_or(0, |f| f.0)
    }
    /// Restore the complete world and discard the superseded future. Failure leaves both untouched.
    pub fn restore(&mut self, id: u32, world: &mut World) -> bool {
        let Some(index) = self.frames.iter().position(|f| f.0 == id) else { return false };
        *world = self.frames[index].2.clone();
        while self.frames.len() > index + 1 {
            if let Some((_, bytes, _)) = self.frames.pop_back() {
                self.bytes -= bytes;
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
        let last = history.frames.back().unwrap().0;
        history.clear();
        assert_eq!(history.oldest(), 0);
        assert!(history.frames.is_empty());
        assert_ne!(history.save(&world), last);
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
