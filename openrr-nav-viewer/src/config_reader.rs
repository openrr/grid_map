use arci::{Localization, MoveBase};
use arci_urdf_viz::UrdfVizWebClient;
use parking_lot::Mutex;
use std::{fs, sync::Arc};
use yaml_rust::YamlLoader;

use crate::{Error, NavigationVizLite};

pub fn navigation_viz_lite_from_yaml_config<M: MoveBase, L: Localization>(
    path: &str,
) -> Result<NavigationVizLite<M, L>, Error> {
    let yaml_file = match fs::read_to_string(path) {
        Ok(s) => s,
        Err(_) => return Err(Error::FailedToLoadConfigFile),
    };
    let config_vec = YamlLoader::load_from_str(&yaml_file).unwrap();
    let config = &config_vec[0];

    let move_base;
    let localization;

    match config["move_base_client"].as_str().unwrap() {
        "UrdfViz" => {
            move_base = Arc::new(Mutex::new({
                let client = UrdfVizWebClient::default();
                client.run_send_velocity_thread();
                client
            }));
        }
        _ => {}
    }

    let mut nav = NavigationVizLite::new(move_base, localization);

    Ok(nav)
}

#[cfg(test)]
mod test {
    use super::*;

    const SAMPLE_YAML_CONFIG_PATH: &str = "./config/example_app.yaml";

    #[test]
    fn test_navigation_viz_lite_yaml_config() {
        // let _ = navigation_viz_lite_from_yaml_config(SAMPLE_YAML_CONFIG_PATH);
    }
}
