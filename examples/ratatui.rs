use antaeus::graphics::{embedded_graphics::DisplayDriver, tui::TerminalDisplay};
use ratatui::{
    Terminal,
    layout::{Constraint, Direction, Layout},
    widgets::{Block, Borders, Paragraph},
};
use vexide::prelude::*;

#[vexide::main]
#[allow(unused_variables)]
async fn main(peripherals: Peripherals) {
    let mut display = DisplayDriver::new(peripherals.display); // Embedded Graphics Display Backend
    let mut terminal = Terminal::from_driver(&mut display);
    let _ = terminal.draw(|frame| {
        // Define screen layout splits adapted for small embedded dimensions
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Top status bar
                Constraint::Min(0),    // Main sensor view
            ])
            .split(frame.area());

        // Top Banner Widget
        let header = Paragraph::new(" Hello World ").block(Block::default().borders(Borders::ALL));
        frame.render_widget(header, chunks[0]);

        // Main Body Widget
        let body = Paragraph::new("Temp: 42°C\nDistance: 1.5m")
            .block(Block::default().title(" Sensors ").borders(Borders::ALL));
        frame.render_widget(body, chunks[1]);
    });
}
