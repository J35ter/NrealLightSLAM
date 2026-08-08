//! Tiny Windows/Linux HID probe: list devices and try opening the Nreal MCU.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn main() {
    let api = match HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("HidApi::new() failed: {e}");
            return;
        }
    };
    println!("=== all HID devices ===");
    for d in api.device_list() {
        println!(
            "  vid={:04x} pid={:04x} usage_page={:04x} usage={:04x} path={:?} product={:?}",
            d.vendor_id(),
            d.product_id(),
            d.usage_page(),
            d.usage(),
            d.path(),
            d.product_string().unwrap_or("?"),
        );
    }
    println!("=== open MCU 0486:573c ===");
    match api.open(0x0486, 0x573c) {
        Ok(_) => println!("MCU open OK"),
        Err(e) => eprintln!("MCU open FAILED: {e:?}"),
    }
    println!("=== open OV580 05a9:0680 ===");
    match api.open(0x05a9, 0x0680) {
        Ok(_) => println!("OV580 open OK"),
        Err(e) => eprintln!("OV580 open FAILED: {e:?}"),
    }
}
