
use rand::prelude::{Distribution, thread_rng};
use rand_distr::Normal;

mod plot;
use plot::{Points, scatter, TimeSeries, timeplot};

fn main() {
    randtest();
   
    vehicle_simulation();

}

const CTRLCYCLE: f64 = 0.1; // 制御周期[s]
const SIMTIME: f64 = 1000.0;   // シミュレーション時間[s]
const SIMLENGTH: usize = (SIMTIME / CTRLCYCLE) as usize; // シミュレーション配列サイズ
const SPEED_AVE: f64 = 1.0; // 平均車速 [m/s]
const SIGMA2_SPD: f64 = 0.1; // 車速の分散
const SIGMA2_SPDSENS: f64 = 1.0; // 速度センサの観測ノイズの分散

fn vehicle_simulation() {
    let mut x_true = TimeSeries::new(CTRLCYCLE, SIMTIME, 0.0);
    let mut x_est = TimeSeries::new(CTRLCYCLE, SIMTIME, 0.0);
    let mut spd_true = TimeSeries::new(CTRLCYCLE, SIMTIME, 0.0);
    let mut rng = thread_rng();
    let spddist = Normal::<f64>::new(0.0, SIGMA2_SPD).unwrap(); // 車速の確率密度関数
    let spdsensdist = Normal::<f64>::new(0.0, SIGMA2_SPDSENS).unwrap(); // 車速の確率密度関数
    let mut xpos = 0.0;
    let mut xpos_est = 0.0;

    for idx in 0..SIMLENGTH - 1 {
        // 車両の実位置
        let spd = (SPEED_AVE) + spddist.sample(&mut rng);
        xpos += CTRLCYCLE * spd;
        
        // センサの観測によるデッドレコニング
        let y = spd + spdsensdist.sample(&mut rng);
        xpos_est += CTRLCYCLE * y;

        x_true.recordvalue(xpos).unwrap(); // x(t+1) = x(t) + Δt * V + noise
        spd_true.recordvalue(spd).unwrap();

        x_est.recordvalue(xpos_est).unwrap();
    }

    let mut maxerr = 0.0;
    let mut maxerr_time = 0.0;
    for idx in 0..SIMLENGTH - 1 {
        let xt = x_true.getvalue_bystep(idx).unwrap();
        let xe = x_est.getvalue_bystep(idx).unwrap();
        let err = (xt - xe).abs();

        if err > maxerr {
            maxerr = err;
            maxerr_time = x_true.gettime_bystep(idx).unwrap();
        }
    }

    let plots_x = vec![x_true, x_est];
    let plots_spd = vec![spd_true];
    timeplot(&plots_x, "x_true.png", "x_true");
    timeplot(&plots_spd, "spd_true.png", "spd_true");

    println!("x_true = {} [m], x_est = {} [m], maxerror = {} [m] @ {} [s]", xpos, xpos_est, maxerr, maxerr_time);
}

fn randtest() {
    let mut points = Points::new(1000);
    let mut rng = thread_rng();
    
    let dist = Normal::<f64>::new(20.0, 5.0).unwrap();
    
    for pt in &mut points.data {
        pt.x = dist.sample(&mut rng);
        pt.y = dist.sample(&mut rng);
    }

    scatter(&points, "normal_dist.png", "normal_dist");
}

