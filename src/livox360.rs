use std::io;
use std::net::UdpSocket;

const IMU_PACKET_BUF_SIZE: usize = 60;
const IMU_PORT: u16 = 56401;

const PCD_PACKET_BUF_SIZE: usize = 1380;
const PCD_PORT: u16 = 56301;
pub const PCD_UDP_COUNT: usize = 209;

/// Handle to an open lidar IMU stream.
pub struct LidarImu {
    sock: UdpSocket,
    buf: [u8; IMU_PACKET_BUF_SIZE],
}

/// One decoded IMU sample.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Timestamp in seconds.
    pub time_stamp: f64,
    /// Raw IMU values: [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z].
    pub imu: [f32; 6],
}

/// Handle to an open lidar PCD stream.
pub struct LidarPcd {
    sock: UdpSocket,
    buf: [u8; PCD_PACKET_BUF_SIZE],
}

/// One decoded PCD sample.
#[derive(Debug, Clone)]
pub struct PcdSample {
    /// Timestamp in seconds.
    pub time_stamp: f64,
    /// Raw PCD values: [x, y, z].
    pub pcd: Vec<[f32; 3]>,
}


/// Bind to the given UDP port and return a handle for reading samples.
pub fn lidar_imu_open() -> io::Result<LidarImu> {
    let sock = UdpSocket::bind(("0.0.0.0", IMU_PORT))?;
    Ok(LidarImu {
        sock,
        buf: [0u8; IMU_PACKET_BUF_SIZE],
    })
}

/// Block until a valid packet arrives and return its decoded sample.
/// Malformed (too short) packets are skipped.
pub fn lidar_imu_read(dev: &mut LidarImu) -> io::Result<ImuSample> {
    loop {
        let (_len, _addr) = dev.sock.recv_from(&mut dev.buf)?;
        if let Some(sample) = parse_imu_packet(&dev.buf) {
            return Ok(sample);
        }
    }
}

/// Roll and pitch in degrees, computed from the accelerometer values.
pub fn imu_roll_pitch(imu: &[f32; 6]) -> (f32, f32) {
    let [acc_x, acc_y, acc_z] = [imu[3], imu[4], imu[5]];

    let roll = acc_y.atan2(acc_z);
    let pitch = (-acc_x).atan2((acc_y * acc_y + acc_z * acc_z).sqrt());

    (roll.to_degrees(), pitch.to_degrees())
}

fn parse_imu_packet(data: &[u8]) -> Option<ImuSample> {
    // Need bytes 28..36 for the timestamp; the last 24 bytes hold the IMU data
    if data.len() < 36 {
        return None;
    }

    let ts_ns = u64::from_le_bytes(data[28..36].try_into().ok()?);
    let time_stamp = ts_ns as f64 / 1e9;

    let imu_bytes = &data[36..];
    let mut imu = [0f32; 6];
    for (i, chunk) in imu_bytes.chunks_exact(4).enumerate() {
        imu[i] = f32::from_le_bytes(chunk.try_into().ok()?);
    }
    Some(ImuSample { time_stamp, imu })
}

/// Bind to the given UDP port and return a handle for reading samples.
pub fn lidar_pcd_open() -> io::Result<LidarPcd> {
    let sock = UdpSocket::bind(("0.0.0.0", PCD_PORT))?;
    Ok(LidarPcd {
        sock,
        buf: [0u8; PCD_PACKET_BUF_SIZE],
    })
}

/// Block until a valid packet arrives and return its decoded sample.
/// Malformed (too short) packets are skipped.
pub fn lidar_pcd_read(dev: &mut LidarPcd) -> io::Result<PcdSample> {
    loop {
        let (_len, _addr) = dev.sock.recv_from(&mut dev.buf)?;
        if let Some(sample) = parse_pcd_packet(&dev.buf) {
            return Ok(sample);
        }
    }
}

fn parse_pcd_packet(data: &[u8]) -> Option<PcdSample> {
    // Need bytes 28..36 for the timestamp; the last 24 bytes hold the IMU data

    if data.len() < 36 {
        return None;
    }

    let ts_ns = u64::from_le_bytes(data[28..36].try_into().ok()?);
    let time_stamp = ts_ns as f64 / 1e9;

    let pcd_bytes = &data[36..];

    const POINT_SIZE: usize = 14;
    const POINT_COUNT: usize = 96;

    let mut points: Vec<[f32; 3]> = Vec::with_capacity(POINT_COUNT);
    //let mut r_values: Vec<f32> = Vec::with_capacity(POINT_COUNT);

    for chunk in pcd_bytes.chunks_exact(POINT_SIZE).take(POINT_COUNT) {
        let x = i32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let y = i32::from_le_bytes(chunk[4..8].try_into().unwrap());
        let z = i32::from_le_bytes(chunk[8..12].try_into().unwrap());
        //let r = u16::from_le_bytes(chunk[12..14].try_into().unwrap());

        points.push([(x as f32) / 10f32, (y as f32) / 10f32, (z as f32) / 10f32]);

    }

    Some(PcdSample { time_stamp, pcd:points })

}