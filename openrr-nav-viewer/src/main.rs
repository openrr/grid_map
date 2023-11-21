use clap::Parser;
use openrr_nav_viewer::*;

#[derive(Debug, Parser)]
struct Args {
    #[clap(
        short = 'f',
        long = "config-file",
        env = "PLANNER_CONFIG_PATH",
        help = "planner config file path"
    )]
    planner_config_path: String,
}

impl TryFrom<Args> for NavigationViz {
    type Error = openrr_nav::Error;

    fn try_from(value: Args) -> Result<Self, Self::Error> {
        NavigationViz::new(&value.planner_config_path)
    }
}

fn main() {
    let nav: NavigationViz = Args::parse().try_into().unwrap();

    let cloned_nav = nav.clone();
    let h = std::thread::spawn(|| {
        let addr = "[::1]:50101".parse().unwrap();
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(
                tonic::transport::Server::builder()
                    .add_service(pb::api_server::ApiServer::new(cloned_nav))
                    .serve(addr),
            )
    });

    let bevy_cloned_nav = nav.clone();
    let mut app = BevyAppNav::new();
    app.setup(bevy_cloned_nav);
    app.run();

    h.join().unwrap().unwrap();
}
