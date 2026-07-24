use antaeus::motion::feedback_control::pid::core_pid::CorePID;
use plotly::{Plot, Scatter, common::Line};

#[test]
//#[ignore = "Long-running visualization test; run explicitly"]
fn large_pid_step_response_plot() {
    let mut pid = CorePID::new(6.0, 0.4, 2.5, 100.0, 60.0, 0.0);

    // RECOMMENDED PID VALUES
    // Kp: 6.0
    // Ki: 0.4
    // Kd: 2.5

    let dt = 0.2_f64;
    let total_time = 20.0_f64;
    let steps = (total_time / dt) as usize;

    let mass = 1.0_f64;
    let damping = 0.5_f64;
    let disturbance = -40.0_f64;
    let threshold = 5.0; // Define your acceptable error threshold

    let mut x = 0.0_f64;
    let mut v = 0.0_f64;

    let mut t_log = Vec::with_capacity(steps);
    let mut x_log = Vec::with_capacity(steps);
    let mut u_log = Vec::with_capacity(steps);
    let mut m_log = Vec::with_capacity(steps);
    let mut target_log = Vec::with_capacity(steps);

    for k in 0..steps {
        let t = k as f64 * dt;
        let u = pid.tick(x, dt);

        let a = (u - damping * v + disturbance) / mass;
        v += a * dt;
        x += v * dt;

        t_log.push(t);
        x_log.push(x);
        u_log.push(u);
        m_log.push(pid.max);
        target_log.push(pid.target);
    }

    let final_error = (pid.target - x).abs();
    assert!(final_error < threshold, "Final error too large: {final_error:.3}");

    let mut plot = Plot::new();

    // Max Power (Dashed Red)
    plot.add_trace(
        Scatter::new(t_log.clone(), m_log.clone())
            .name("Maximum Motor Power (arbitrary units)")
            .line(
                Line::new()
                    .color("rgba(214, 39, 40, 0.8)")
                    .dash(plotly::common::DashType::Dash),
            ),
    );

    // Target (Dotted Blue)
    plot.add_trace(
        Scatter::new(t_log.clone(), target_log.clone())
            .name("Target (degrees)")
            .line(
                Line::new()
                    .color("#1f77b4")
                    .dash(plotly::common::DashType::Dot),
            ),
    );

    // Position (Solid Orange)
    plot.add_trace(
        Scatter::new(t_log.clone(), x_log.clone())
            .name("Sensor Reading (degrees)")
            .line(Line::new().color("orange")),
    );

    // Control Output (Solid Green)
    plot.add_trace(
        Scatter::new(t_log.clone(), u_log.clone())
            .name("Motor Output (arbitrary units)")
            .line(Line::new().color("rgba(44, 160, 44, 1.0)")),
    );

    std::fs::create_dir_all("test-artifacts/test-plots").expect("create plot dir failed");
    let out_file = "test-artifacts/test-plots/large_pid_step_response.html";
    plot.write_html(out_file);

    println!("Plot written: {out_file}");

    // --- Export Simulation Data to CSV ---
    let csv_file_path = "test-artifacts/test-plots/large_pid_step_response.csv";
    let file = std::fs::File::create(csv_file_path).expect("Failed to create CSV file");
    let mut wtr = csv::Writer::from_writer(file);

    // Write the header row
    wtr.write_record(&[
        "Time (s)",
        "Sensor Reading (deg)",
        "Motor Output (u)",
        "Max Power",
        "Target (deg)",
    ])
    .expect("Failed to write CSV header");

    // Write data rows
    for k in 0..steps {
        wtr.serialize((t_log[k], x_log[k], u_log[k], m_log[k], target_log[k]))
            .expect("Failed to write CSV row");
    }

    wtr.flush().expect("Failed to flush CSV writer");
    println!("CSV data written to: {csv_file_path}");
}
