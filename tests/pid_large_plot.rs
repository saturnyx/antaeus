use antaeus::motion::feedback_control::pid::core_pid::CorePID;

#[test]
#[ignore = "Long-running visualization test; run explicitly"]
fn large_pid_step_response_plot() {
    use plotly::{Plot, Scatter};

    let mut pid = CorePID::new(2.0, 0.25, 0.5, 100.0, 60.0, 0.25);

    let dt = 0.01_f64;
    let total_time = 60.0_f64;
    let steps = (total_time / dt) as usize;

    let mass = 1.0_f64;
    let damping = 1.2_f64;

    let mut x = 0.0_f64;
    let mut v = 0.0_f64;

    let mut t_log = Vec::with_capacity(steps);
    let mut x_log = Vec::with_capacity(steps);
    let mut u_log = Vec::with_capacity(steps);
    let mut target_log = Vec::with_capacity(steps);

    for k in 0..steps {
        let t = k as f64 * dt;
        let u = pid.tick(x, dt);

        let a = (u - damping * v) / mass;
        v += a * dt;
        x += v * dt;

        t_log.push(t);
        x_log.push(x);
        u_log.push(u);
        target_log.push(pid.target);
    }

    let final_error = (pid.target - x).abs();
    assert!(final_error < 5.0, "Final error too large: {final_error:.3}");

    let mut plot = Plot::new();
    plot.add_trace(Scatter::new(t_log.clone(), x_log).name("position"));
    plot.add_trace(Scatter::new(t_log.clone(), target_log).name("target"));
    plot.add_trace(Scatter::new(t_log, u_log).name("control_output"));

    std::fs::create_dir_all("target/test-plots").expect("create plot dir failed");
    let out_file = "target/test-plots/large_pid_step_response.html";
    plot.write_html(out_file);

    println!("Plot written: {out_file}");
}
