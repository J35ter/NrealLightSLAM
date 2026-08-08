//! HID probe: replicate NrealLight::new() exactly — MCU commands then OV580 init.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn mcu_cmd(dev: &hidapi::HidDevice, cat: u8, id: u8, data: &[u8]) -> Result<Vec<u8>, String> {
    let mut pkt = [0u8; 64];
    pkt[0] = 2; pkt[1] = b':'; pkt[2] = cat; pkt[3] = b':'; pkt[4] = id; pkt[5] = b':';
    let n = data.len().min(50);
    pkt[6..6 + n].copy_from_slice(&data[..n]);
    let rest = b":0:00000000:3";
    pkt[6 + n..6 + n + rest.len()].copy_from_slice(rest);
    dev.write(&pkt).map_err(|e| format!("write {e:?}"))?;
    for i in 0..64 {
        let mut buf = [0u8; 64];
        match dev.read_timeout(&mut buf, 250) {
            Ok(0) => return Err("timeout read 0".into()),
            Ok(_) => {
                if buf[0] == 2 && buf[2] == cat + 1 {
                    return Ok(buf.to_vec());
                }
                // else skip
            }
            Err(e) => return Err(format!("read err {e:?}")),
        }
    }
    Err("no ack in 64".into())
}

fn main() {
    let api = HidApi::new().unwrap();
    let mcu = api.open(0x0486, 0x573c).expect("MCU open");
    println!("MCU open OK");
    println!("MCU @3 (SDK)  -> {:?}", mcu_cmd(&mcu, b'@', b'3', &[b'1']).map(|v| v[..6].to_vec()));
    println!("MCU 1L (amb)  -> {:?}", mcu_cmd(&mcu, b'1', b'L', &[b'1']).map(|v| v[..6].to_vec()));
    println!("MCU 1N (vsync)-> {:?}", mcu_cmd(&mcu, b'1', b'N', &[b'1']).map(|v| v[..6].to_vec()));
    let ov = api.open(0x05a9, 0x0680).expect("OV580 open");
    println!("OV580 open OK");
    // then OV580 init (0x19,0x0 etc.) — brief
    println!("OV580 0x19,0x0 -> {}", {
        ov.write(&[2, 0x19, 0x0, 0, 0, 0, 0]).is_ok()
    });
}
