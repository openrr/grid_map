fn main() {
    println!("cargo:rerun-if-changed=proto");

    let config = tonic_build::configure();
    config
        .compile(&["proto/openrr_nav_viewer.proto"], &["proto"])
        .unwrap();
}
