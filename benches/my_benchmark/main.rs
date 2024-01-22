use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::fs;




fn benchmark(c: &mut Criterion){
    // let scan1 = parse_scan("./scan1.txt");
    // let scan2 = parse_scan("./scan2.txt");
    // let mut group = c.benchmark_group("Icp");
    // let mut icp = Icp::new(scan1, scan2);

    // group.bench_function("icp_linear", |b|{
    //     b.iter(||icp.do_icp(0.0, 0.0));
    // });

    // group.bench_function("icp_kdtree", |b|{
    //     b.iter(||icp.do_icp_fast(0.0, 0.0));
    // });

}

criterion_group!(benches,benchmark);
criterion_main!(benches);