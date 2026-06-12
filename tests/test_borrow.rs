use passiveradar::ui::dashboard::Dashboard;
use ratatui::layout::Rect;
use std::time::Instant;

#[test]
fn test_borrow_speed() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
    for _ in 0..200 { db.add_spectrum(vec![0.0; 1000]); }
    let area = Rect::new(0, 0, 300, 300);
    let mut backend = ratatui::backend::TestBackend::new(300, 300);
    let mut terminal = ratatui::Terminal::new(backend).unwrap();
    terminal.draw(|f| { db.render(f, area, &[], &[]); }).unwrap();

    let t = Instant::now();
    for _ in 0..100 {
        let _borrowed_lines: Vec<ratatui::text::Line> = db.cached_waterfall_lines.iter().map(|line| {
            ratatui::text::Line {
                spans: line.spans.iter().map(|span| {
                    ratatui::text::Span {
                        content: std::borrow::Cow::Borrowed(span.content.as_ref()),
                        style: span.style,
                    }
                }).collect(),
                ..line.clone() // wait, the current code has `..*line`
            }
        }).collect();
    }
    println!("100 borrows took: {:?}", t.elapsed());
}
