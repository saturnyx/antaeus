use std::path::Path;

use plotly::{
    Layout,
    Plot,
    Scatter,
    Traces,
    common::{DashType, Line, Marker, Mode, Title},
    layout::{
        Animation,
        AnimationMode,
        AnimationOptions,
        Axis,
        Frame,
        FrameSettings,
        Slider,
        SliderCurrentValue,
        SliderStepBuilder,
        TransitionSettings,
        update_menu::{ButtonBuilder, UpdateMenu, UpdateMenuDirection, UpdateMenuType},
    },
};

const MAX_PLOT_FRAMES: usize = 600;

/// A single recorded robot pose sample used for CSV export and Plotly replay.
#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct TraceSample {
    pub step:        usize,
    pub time_s:      f64,
    pub segment:     String,
    pub pose_x_in:   f64,
    pub pose_y_in:   f64,
    pub heading_rad: f64,
}

/// Renders an interactive Plotly HTML artifact directly from the trace data.
///
/// The generated plot contains:
/// - a dashed green requested path underlay, if provided;
/// - the estimated robot path in blue;
/// - a moving orange robot marker;
/// - a slider and play/pause controls over animation frames.
pub fn write_trace_plot_html(
    samples: &[TraceSample],
    planned_path: &[(f64, f64)],
    html_path: &str,
    title: &str,
) {
    assert!(!samples.is_empty(), "plot trace requires at least one sample");

    let plot_samples = decimate_samples(samples, MAX_PLOT_FRAMES);

    let mut plot = Plot::new();

    if planned_path.len() >= 2 {
        plot.add_trace(
            Scatter::new(
                planned_path.iter().map(|(x, _)| *x).collect::<Vec<_>>(),
                planned_path.iter().map(|(_, y)| *y).collect::<Vec<_>>(),
            )
            .name("Requested path")
            .mode(Mode::Lines)
            .line(Line::new().color("#66d17a").dash(DashType::Dash).width(3.0)),
        );
    }

    let first = &plot_samples[0];
    plot.add_trace(
        Scatter::new(vec![first.pose_x_in], vec![first.pose_y_in])
            .name("Estimated path")
            .mode(Mode::Lines)
            .line(Line::new().color("#4aa3ff").width(4.0)),
    );
    plot.add_trace(
        Scatter::new(vec![first.pose_x_in], vec![first.pose_y_in])
            .name("Robot")
            .mode(Mode::Markers)
            .marker(Marker::new().size(14).color("#ffb84a")),
    );

    let animation_options = AnimationOptions::new()
        .frame(FrameSettings::new().duration(12).redraw(true))
        .transition(TransitionSettings::new().duration(0))
        .mode(AnimationMode::Immediate);

    for (frame_index, sample) in plot_samples.iter().enumerate() {
        let upto = &plot_samples[..=frame_index];
        let frame_name = frame_name(frame_index);
        let mut traces = Traces::new();
        traces.push(
            Scatter::new(
                upto.iter().map(|row| row.pose_x_in).collect::<Vec<_>>(),
                upto.iter().map(|row| row.pose_y_in).collect::<Vec<_>>(),
            )
            .mode(Mode::Lines)
            .line(Line::new().color("#4aa3ff").width(4.0)),
        );
        traces.push(
            Scatter::new(vec![sample.pose_x_in], vec![sample.pose_y_in])
                .mode(Mode::Markers)
                .marker(Marker::new().size(14).color("#ffb84a")),
        );
        let frame = Frame::new()
            .name(frame_name.clone())
            .traces(vec![1, 2])
            .data(traces);
        plot.add_frame(frame);
    }

    let slider_steps = plot_samples
        .iter()
        .enumerate()
        .map(|(frame_index, sample)| {
            SliderStepBuilder::new()
                .label(format!("{:.2}s", sample.time_s))
                .animation(
                    Animation::frames(vec![frame_name(frame_index)])
                        .options(animation_options.clone()),
                )
                .build()
                .expect("build Plotly slider step")
        })
        .collect::<Vec<_>>();

    let slider = Slider::new()
        .active(0)
        .current_value(SliderCurrentValue::new().prefix("Sample: "))
        .steps(slider_steps)
        .x(0.12)
        .y(-0.08)
        .length(0.78);

    let play_button = ButtonBuilder::new()
        .label("Play")
        .animation(Animation::all_frames().options(animation_options.clone().fromcurrent(true)))
        .build()
        .expect("build Plotly play button");
    let pause_button = ButtonBuilder::new()
        .label("Pause")
        .animation(Animation::pause())
        .build()
        .expect("build Plotly pause button");
    let buttons = UpdateMenu::new()
        .ty(UpdateMenuType::Buttons)
        .direction(UpdateMenuDirection::Left)
        .show_active(false)
        .x(0.0)
        .y(-0.08)
        .buttons(vec![play_button, pause_button]);

    let (min_x, max_x, min_y, max_y) = bounds(samples, planned_path);
    let layout = Layout::new()
        .title(Title::with_text(title))
        .x_axis(
            Axis::new()
                .title(Title::with_text("X (in)"))
                .range(vec![min_x, max_x])
                .show_grid(true)
                .zero_line(true),
        )
        .y_axis(
            Axis::new()
                .title(Title::with_text("Y (in)"))
                .range(vec![min_y, max_y])
                .show_grid(true)
                .zero_line(true)
                .scale_anchor("x"),
        )
        .sliders(vec![slider])
        .update_menus(vec![buttons]);
    plot.set_layout(layout);

    let parent = Path::new(html_path)
        .parent()
        .expect("plot HTML has parent directory");
    std::fs::create_dir_all(parent).expect("create Plotly output directory");
    plot.write_html(html_path);
}

fn frame_name(step: usize) -> String { format!("frame_{step}") }

fn decimate_samples(samples: &[TraceSample], max_frames: usize) -> Vec<TraceSample> {
    if samples.len() <= max_frames {
        return samples.to_vec();
    }

    let stride = samples.len().div_ceil(max_frames);
    let mut reduced = samples.iter().step_by(stride).cloned().collect::<Vec<_>>();
    if reduced.last().map(|sample| sample.step) != samples.last().map(|sample| sample.step) {
        reduced.push(samples.last().expect("trace has a last sample").clone());
    }
    reduced
}

fn bounds(samples: &[TraceSample], planned_path: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let mut xs = samples
        .iter()
        .map(|sample| sample.pose_x_in)
        .collect::<Vec<_>>();
    let mut ys = samples
        .iter()
        .map(|sample| sample.pose_y_in)
        .collect::<Vec<_>>();
    xs.extend(planned_path.iter().map(|(x, _)| *x));
    ys.extend(planned_path.iter().map(|(_, y)| *y));
    xs.push(0.0);
    ys.push(0.0);

    let min_x = xs.iter().copied().fold(f64::INFINITY, f64::min);
    let max_x = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min_y = ys.iter().copied().fold(f64::INFINITY, f64::min);
    let max_y = ys.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let padding = ((max_x - min_x).abs().max((max_y - min_y).abs()) * 0.1).max(1.0);

    (min_x - padding, max_x + padding, min_y - padding, max_y + padding)
}
