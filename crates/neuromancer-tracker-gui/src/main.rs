//! `neuromancer-tracker-gui` — Linux + Windows GUI for the Nreal Light
//! head tracker.
//!
//! Embeds the minimal IMU core from `neuromancer-tracker` (lib): the IMU
//! runs on a background thread (`ImuTracker::next_pose` → Mahony), and the
//! egui UI shows:
//!
//! - a **Settings menu** whose items mirror the CLI switches
//!   (`--no-udp`, `--host`, `--port`, `--udp-rate`, `--hud-rate`,
//!   `--gyro-calib`, `--kp`, `--ki`, `--units`), persisted to the same
//!   TOML settings file the CLI reads/writes (`Settings::config_path`);
//! - a **HUD** (YPR readout + IMU rate);
//! - an optional **3D wireframe cube** visualizing the glasses rotation
//!   (toggle on/off), drawn with painter-side 3D→2D projection (no GPU
//!   dependency beyond what egui already uses).
//!
//! The rotation quaternion comes from `neuromancer_ahrs::Quat` (world→body,
//! same convention as the tracker).

use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use eframe::egui;
use neuromancer_tracker::{ImuTracker, Pose, Settings, UdpSink};

// ---------------------------------------------------------------------------
// Shared state between the IMU thread and the UI
// ---------------------------------------------------------------------------

#[derive(Default)]
struct LiveState {
    pose: Option<Pose>,
    rate_hz: f64,
    calibrating: bool,
    error: Option<String>,
}

enum ImuMsg {
    Pose(Pose),
    Rate(f64),
    Calibrating(bool),
    Error(String),
}

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------

struct TrackerApp {
    settings: Settings,
    live: Arc<Mutex<LiveState>>,
    tx: mpsc::Sender<ImuMsg>,
    rx: mpsc::Receiver<ImuMsg>,
    udp: Option<UdpSink>,
    running: bool,
    last_settings_note: Option<String>,
}

impl TrackerApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let settings = Settings::load();
        let (tx, rx) = mpsc::channel::<ImuMsg>();

        // UDP sink (created once; rebuilt when settings change).
        let udp = if settings.no_udp {
            None
        } else {
            UdpSink::bind(&settings.host, settings.port, settings.udp_rate).ok()
        };

        let mut app = TrackerApp {
            settings,
            live: Arc::new(Mutex::new(LiveState::default())),
            tx,
            rx,
            udp,
            running: false,
            last_settings_note: None,
        };
        let _ = cc; // theme/fonts later if needed
        app.start_tracker();
        app
    }

    fn start_tracker(&mut self) {
        let settings = self.settings.clone();
        let tx = self.tx.clone();
        std::thread::spawn(move || {
            let mut tracker = match ImuTracker::open() {
                Ok(t) => t,
                Err(e) => {
                    tx.send(ImuMsg::Error(e)).ok();
                    return;
                }
            };
            tracker.configure(&settings);
            let calibrating = settings.gyro_calib > 0.0;
            tx.send(ImuMsg::Calibrating(calibrating)).ok();
            loop {
                match tracker.next_pose() {
                    Ok(None) => {
                        if calibrating && !tracker.calibrating() {
                            tx.send(ImuMsg::Calibrating(false)).ok();
                        }
                        continue;
                    }
                    Ok(Some(pose)) => {
                        tx.send(ImuMsg::Pose(pose)).ok();
                        tx.send(ImuMsg::Rate(tracker.last_rate_hz)).ok();
                    }
                    Err(e) => {
                        tx.send(ImuMsg::Error(e)).ok();
                        break;
                    }
                }
            }
        });
        self.running = true;
    }
}

impl eframe::App for TrackerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // The IMU thread streams poses continuously; egui only repaints when
        // told to, so request a repaint every frame (and again shortly
        // after) — otherwise the HUD/cube freeze between input events,
        // appearing to "run ~0.5 s then hang ~1 s" in bursts.
        ctx.request_repaint();
        ctx.request_repaint_after(std::time::Duration::from_millis(16)); // ~60 fps

        // Drain the IMU thread's messages.
        while let Ok(msg) = self.rx.try_recv() {
            match msg {
                ImuMsg::Pose(p) => {
                    // Send to Opentrack UDP (rate-gated inside the sink).
                    if let Some(s) = self.udp.as_mut() {
                        s.send(if self.settings.rad { p.ypr_rad } else { p.ypr_rad.map(f64::to_degrees) });
                    }
                    self.live.lock().unwrap().pose = Some(p);
                }
                ImuMsg::Rate(r) => self.live.lock().unwrap().rate_hz = r,
                ImuMsg::Calibrating(c) => self.live.lock().unwrap().calibrating = c,
                ImuMsg::Error(e) => self.live.lock().unwrap().error = Some(e),
            }
        }

        egui::TopBottomPanel::top("menu").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Nreal Light Tracker");
                ui.separator();
                if ui.button("Start").clicked() && !self.running {
                    self.start_tracker();
                }
                if ui.button("Stop").clicked() {
                    self.running = false;
                    // Stopping the USB thread isn't instantaneous; a full stop
                    // would need a cancel token. For now Stop just detaches
                    // from the UI state (the thread exits on USB error).
                }
                ui.separator();
                settings_menu(ui, &mut self.settings, &mut self.last_settings_note);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let live = self.live.lock().unwrap();

            // Error banner
            if let Some(e) = &live.error {
                ui.colored_label(egui::Color32::RED, format!("IMU error: {e}"));
            }
            if live.calibrating {
                ui.colored_label(egui::Color32::YELLOW, "gyro bias calibration: keep the device still ...");
            }

            // HUD
            if let Some(pose) = live.pose {
                let (yaw, pitch, roll) = if self.settings.rad {
                    (pose.ypr_rad[0], pose.ypr_rad[1], pose.ypr_rad[2])
                } else {
                    (
                        pose.ypr_rad[0].to_degrees(),
                        pose.ypr_rad[1].to_degrees(),
                        pose.ypr_rad[2].to_degrees(),
                    )
                };
                let unit = if self.settings.rad { "rad" } else { "°" };
                ui.label(format!("YAW {yaw:8.3} {unit}   PITCH {pitch:8.3} {unit}   ROLL {roll:8.3} {unit}"));
            } else {
                ui.label("no pose yet");
            }
            ui.label(format!("IMU rate: {:.0} Hz", live.rate_hz));
            drop(live);

            if let Some(note) = &self.last_settings_note {
                ui.label(note.clone());
            }

            // 3D cube visualization (toggle).
            if self.settings.show_cube {
                ui.separator();
                ui.label("Glasses rotation (cube):");
                let live = self.live.lock().unwrap();
                let pose = live.pose;
                drop(live);
                cube_panel(ui, pose.map(|p| p.q));
            }
        });
    }
}

// ---------------------------------------------------------------------------
// Settings menu — one widget per CLI switch.
// ---------------------------------------------------------------------------

fn settings_menu(ui: &mut egui::Ui, settings: &mut Settings, note: &mut Option<String>) {
    let mut save = false;
    ui.menu_button("Settings", |ui| {
        ui.checkbox(&mut settings.no_udp, "Disable UDP (--no-udp)");
        ui.add(egui::DragValue::new(&mut settings.udp_rate).prefix("UDP rate (--udp-rate): ").range(1.0..=200.0));
        ui.add(egui::DragValue::new(&mut settings.hud_rate).prefix("HUD rate (--hud-rate): ").range(0.0..=30.0));
        ui.add(egui::DragValue::new(&mut settings.gyro_calib).prefix("Gyro calib (--gyro-calib): ").range(0.0..=10.0));
        ui.add(egui::DragValue::new(&mut settings.kp).prefix("Mahony kp (--kp): ").range(0.0..=5.0));
        ui.add(egui::DragValue::new(&mut settings.ki).prefix("Mahony ki (--ki): ").range(0.0..=0.1));
        ui.horizontal(|ui| {
            ui.label("Units (--units):");
            ui.radio_value(&mut settings.rad, false, "deg");
            ui.radio_value(&mut settings.rad, true, "rad");
        });
        ui.horizontal(|ui| {
            ui.label("UDP host (--host):");
            ui.text_edit_singleline(&mut settings.host);
        });
        ui.add(egui::DragValue::new(&mut settings.port).prefix("UDP port (--port): ").range(1..=65535));
        ui.checkbox(&mut settings.show_cube, "Show 3D cube visualization");
        ui.separator();
        if ui.button("Save settings (TOML)").clicked() {
            save = true;
        }
    });
    if save {
        match settings.save() {
            Ok(path) => {
                *note = Some(format!("settings saved to {}", path.display()));
            }
            Err(e) => *note = Some(format!("settings save failed: {e}")),
        }
    }
}

// ---------------------------------------------------------------------------
// 3D wireframe cube — painter-side projection, no GPU code of our own.
// ---------------------------------------------------------------------------

/// Draw a wireframe cube rotated by `q` (world→body quaternion), axes
/// colored, plus a ground plane hint. Uses egui's Painter with a simple
/// orthographic projection of the 8 cube corners.
fn cube_panel(ui: &mut egui::Ui, q: Option<neuromancer_ahrs::Quat>) {
    let desired = egui::vec2(260.0, 260.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let center = rect.center();

    // Cube corners in body coordinates (unit cube, half-extent 0.6).
    let corners = [
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ];
    // Cube edges (index pairs).
    let edges = [
        (0, 1), (1, 2), (2, 3), (3, 0), // back face
        (4, 5), (5, 6), (6, 7), (7, 4), // front face
        (0, 4), (1, 5), (2, 6), (3, 7), // connectors
    ];

    let q = q.unwrap_or(neuromancer_ahrs::Quat::new(1.0, 0.0, 0.0, 0.0));

    // Project: rotate each corner by q, then a fixed view rotation so we
    // see the cube from a 3/4 angle, then orthographic (x→right, y→down).
    let view = view_rotation();
    let projected: Vec<egui::Pos2> = corners
        .iter()
        .map(|c| {
            let v = q.rotate_vector(*c);
            let v = view_rotate(view, v);
            let s = 55.0; // scale (px per unit)
            egui::pos2(center.x + (v[0] * s) as f32, center.y - (v[1] * s) as f32)
        })
        .collect();

    // Depth cue: edges on the far side get a dimmer color. Simple heuristic:
    // use the average z of the two endpoints.
    for (a, b) in edges {
        let za = corners[a][2];
        let zb = corners[b][2];
        let depth = (za + zb) / 2.0;
        let color = if depth < 0.0 {
            egui::Color32::from_rgb(120, 120, 140)
        } else {
            egui::Color32::from_rgb(200, 200, 220)
        };
        painter.line_segment([projected[a], projected[b]], egui::Stroke::new(1.5_f32, color));
    }

    // Axis arrows (body axes after rotation): X red, Y green, Z blue.
    let axes: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let colors = [
        egui::Color32::from_rgb(230, 80, 80),
        egui::Color32::from_rgb(80, 220, 80),
        egui::Color32::from_rgb(80, 130, 240),
    ];
    for (axis, color) in axes.iter().zip(colors.iter()) {
        let v = q.rotate_vector(*axis);
        let v = view_rotate(view, v);
        let end = egui::pos2(
            center.x + (v[0] * 75.0) as f32,
            center.y - (v[1] * 75.0) as f32,
        );
        painter.line_segment([center, end], egui::Stroke::new(2.0_f32, *color));
    }

    // Caption.
    painter.text(
        rect.left_bottom() + egui::vec2(2.0, 12.0),
        egui::Align2::LEFT_BOTTOM,
        format!("q = ({:.3}, {:.3}, {:.3}, {:.3})", q.w, q.x, q.y, q.z),
        egui::FontId::monospace(10.0),
        egui::Color32::GRAY,
    );
}

/// Fixed 3/4 view rotation: rotate the cube so we see three faces.
fn view_rotation() -> [[f64; 3]; 3] {
    // Rotation matrix: R = Rx(0.5)·Ry(-0.6) (about x then y).
    let (a, b) = (0.5f64, -0.6f64);
    let (ca, sa) = (a.cos(), a.sin());
    let (cb, sb) = (b.cos(), b.sin());
    [
        [cb, 0.0, sb],
        [sa * sb, ca, -sa * cb],
        [-ca * sb, sa, ca * cb],
    ]
}

fn view_rotate(r: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Nreal Light Tracker")
            .with_inner_size([480.0, 420.0]),
        ..Default::default()
    };
    eframe::run_native(
        "neuromancer-tracker-gui",
        options,
        Box::new(|cc| Ok(Box::new(TrackerApp::new(cc)))),
    )
}
