//! Print the derivation JSON of a preset: `cargo run --example derive -- cartpole`.
use motion_core::mech::presets;
use motion_core::model::Model;
fn main() {
    let name = std::env::args().nth(1).unwrap_or_else(|| "cartpole".into());
    let d = match name.as_str() {
        "arm2" => presets::arm2(),
        "acrobot" => presets::acrobot(),
        "double" => presets::double_pendulum(),
        "fourbar" => presets::four_bar(),
        _ => presets::cartpole(),
    };
    println!("{}", Model::from_desc(&d).derivation);
}
