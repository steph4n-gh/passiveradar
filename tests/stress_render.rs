use passiveradar::ui::dashboard::Dashboard;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct Allocator;

#[global_allocator]
static ALLOCATOR: Allocator = Allocator;
static ALLOCATED: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATED.fetch_add(layout.size(), Ordering::SeqCst);
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        ALLOCATED.fetch_sub(layout.size(), Ordering::SeqCst);
        System.dealloc(ptr, layout)
    }
}

#[test]
fn test_render_memory_leak() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
    let backend = TestBackend::new(200, 200);
    let mut terminal = Terminal::new(backend).unwrap();

    let targets = vec![];
    let transients = vec![];

    // warm up
    for _ in 0..100 {
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();
    }

    let initial_allocated = ALLOCATED.load(Ordering::SeqCst);

    for _ in 0..10_000 {
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();
    }

    let final_allocated = ALLOCATED.load(Ordering::SeqCst);

    let diff = if final_allocated > initial_allocated { final_allocated - initial_allocated } else { 0 };
    println!("Memory diff after 10000 renders: {} bytes", diff);
    // Allowing a small buffer for cache resizing, but it shouldn't grow boundlessly.
    assert!(diff < 5 * 1024 * 1024, "Memory grew by more than 5MB: {} bytes", diff);
}
