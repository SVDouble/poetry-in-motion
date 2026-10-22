//! Checkpoint storage estimates. Count allocation capacities, excluding allocator bookkeeping and shared code.
use std::sync::Arc;

/// Vector elements only; the owner already accounts for the inline Vec header.
pub fn vec_bytes<T>(values: &Vec<T>) -> usize {
    values.capacity() * size_of::<T>()
}

/// Shared allocations are charged in full to each checkpoint, including a reference-count header.
pub fn arc_bytes<T: ?Sized>(value: &Arc<T>) -> usize {
    size_of_val(value.as_ref()) + size_of::<[usize; 2]>()
}
