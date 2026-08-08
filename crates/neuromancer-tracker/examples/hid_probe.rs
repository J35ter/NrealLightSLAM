//! HID probe: replicate read_event() exactly — MCU read(0) then OV read.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use std::time::Instant;
use hidapi::HidApi;

fn main() {
    let api = HidApi::new().unwrap();
    let mcu = api.open(0x0486, 0x573c).unwrap();
    let ov = api.open(0x05a9, 0x0680).unwrap();
    ov.write(&[2, 0x19, 0x1, 0, 0, 0, 0]).unwrap();

    println!("iter\tMCU_read0(ms)\tOV_read(ms)\tOV_bytes");
    for i in 0..30 {
        // read_mcu_packet: read_packet(0) — MCU read with timeout 0
        let mut mbuf = [0u8; 0x40];
        let m0 = Instant::now();
        let mn = mcu.read_timeout(&mut mbuf, 0).unwrap_or(0);
        let m_ms = m0.elapsed().as_secs_f64() * 1000.0;

        // ov580.read_packet: loop read until buf[0]==1
        let mut obuf = [0u8; 0x80];
        let o0 = Instant::now();
        let on = loop {
            match ov.read_timeout(&mut obuf, 250) {
                Ok(n) if n > 0 && obuf[0] == 1 => break n,
                Ok(_) => continue,
                Err(_) => break 0,
            }
        };
        let o_ms = o0.elapsed().as_secs_f64() * 1000.0;
        println!("{i}\t{m_ms:.1}\t\t{o_ms:.1}\t\t{on}");
    }
}
