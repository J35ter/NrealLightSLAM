//! HID probe: run the real NrealLight::new() + read IMU events (Windows debug).
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use ar_drivers::ARGlasses;

fn main() {
    println!("=== NrealLight::new() (non-fatal MCU init) ===");
    let start = std::time::Instant::now();
    match ar_drivers::nreal_light::NrealLight::new() {
        Ok(mut g) => {
            println!("OK after {:.1}s — reading events (10 s)...", start.elapsed().as_secs_f64());
            let mut got = 0;
            let t0 = std::time::Instant::now();
            while t0.elapsed().as_secs_f64() < 10.0 {
                match g.read_event() {
                    Ok(ev) => {
                        got += 1;
                        let brief: String = format!("{ev:?}").chars().take(90).collect();
                        println!("  event#{got}: {brief}");
                        if got >= 5 {
                            break;
                        }
                    }
                    Err(e) => {
                        println!("  read_event err: {e:?}");
                        break;
                    }
                }
            }
            println!("events received: {got}");
        }
        Err(e) => eprintln!("NrealLight::new() FAILED: {e:?}"),
    }
}
