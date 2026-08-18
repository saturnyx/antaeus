// examples/ratatui.rs — identical for vexos and host
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
    let mut display = DisplayDriver::new(peripherals.display);
    let mut terminal = Terminal::from_driver(&mut display);

    loop {
        let _ = terminal.draw(|frame| {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([Constraint::Length(3), Constraint::Min(0)])
                .split(frame.area());

            let header =
                Paragraph::new(" Hello World ").block(Block::default().borders(Borders::ALL));
            frame.render_widget(header, chunks[0]);

            let body = Paragraph::new("Temp: 42°C\nDistance: 1.5m")
                .block(Block::default().title(" Sensors ").borders(Borders::ALL));
            frame.render_widget(body, chunks[1]);
        });
    }
}
