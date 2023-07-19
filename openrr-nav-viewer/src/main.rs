use openrr_nav_viewer::*;

fn main() {
    let nav = NavigationViz::default();

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
