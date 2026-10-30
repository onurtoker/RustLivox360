mod livox360;

use livox360::{lidar_imu_open, lidar_imu_read, imu_roll_pitch};
use livox360::{lidar_pcd_open, lidar_pcd_read, lidar_pcd_capture};

use std::time::{Duration, Instant};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::thread;

fn log_imu_thread(path: &str, sample_count: usize) {
    let mut imu_dev = lidar_imu_open().unwrap();
    let mut out = BufWriter::new(File::create(path).unwrap());

    for _ in 0..sample_count {
        let sample = lidar_imu_read(&mut imu_dev).unwrap();
        let (roll, pitch) = imu_roll_pitch(&sample.imu);
        writeln!(out, "{:.6}, {:.2}, {:.2}, {:.6}, {:.6}, {:.6}",
                 sample.time_stamp, roll, pitch,
                 sample.imu[0], sample.imu[1], sample.imu[2]).unwrap();
    }

    out.flush().unwrap();
}

fn log_pcd_thread(path: &str, packet_count: usize) {
    let mut pcd_dev = lidar_pcd_open().unwrap();
    let mut out = BufWriter::new(File::create(path).unwrap());

    for _ in 0..packet_count {
        let sample = lidar_pcd_read(&mut pcd_dev).unwrap();
        for [x, y, z] in &sample.pcd {
            writeln!(out, "{}, {},{},{}", sample.time_stamp, x, y, z).unwrap();
        }
    }
    out.flush().unwrap();
}

fn main_file_test() {
    let handle_imu = thread::spawn(||
        log_imu_thread("imu_log.txt", 5 * 200));
    let handle_pcd = thread::spawn(||
        log_pcd_thread("pcd_log.txt", livox360::PCD_UDP_COUNT * 5 * 10));

    handle_pcd.join().unwrap();
    handle_imu.join().unwrap();
}

fn main_read_test() {
    let mut pcd_dev = lidar_pcd_open().unwrap();

    let start = Instant::now();
    let mut window_start = start;
    let mut total_calls: u64 = 0;
    let mut window_calls: u64 = 0;
    let report_every = Duration::from_secs(10);

    loop {
        let points = lidar_pcd_capture(&mut pcd_dev).unwrap();
        // use `points` here; keep it alive so the call isn't optimized away
        std::hint::black_box(&points);

        total_calls += 1;
        window_calls += 1;

        let now = Instant::now();
        let window_elapsed = now - window_start;
        if window_elapsed >= report_every {
            let avg_total = total_calls as f64 / (now - start).as_secs_f64();
            let avg_window = window_calls as f64 / window_elapsed.as_secs_f64();
            println!(
                "calls/s: {:.2} (last {:.1}s), {:.2} (overall avg), total calls: {}",
                avg_window,
                window_elapsed.as_secs_f64(),
                avg_total,
                total_calls
            );
            window_start = now;
            window_calls = 0;
        }
    }
}

fn main() {
    println!("Livox360 test code");
}