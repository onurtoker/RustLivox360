mod livox360;

use livox360::{lidar_imu_open, lidar_imu_read, imu_roll_pitch};
use livox360::{lidar_pcd_open, lidar_pcd_read};

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

fn main() {
    let handle_imu = thread::spawn(||
        log_imu_thread("imu_log.txt", 5 * 200));
    let handle_pcd = thread::spawn(||
        log_pcd_thread("pcd_log.txt", livox360::PCD_UDP_COUNT * 5 * 10));

    handle_pcd.join().unwrap();
    handle_imu.join().unwrap();
}