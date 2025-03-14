#![no_std]
#![no_main]
use cortex_m_rt::entry;
// use defmt::{info, println};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::{Point, Primitive}, primitives::{Line, PrimitiveStyle}};
use libm::{cosf, sinf};
use panic_halt as _;
use ssd1306::{mode::DisplayConfig, prelude::DisplayRotation, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    flash::FlashExt, gpio::GpioExt, i2c::{BlockingI2c, DutyCycle, Mode}, pac::Peripherals, prelude::*, rcc::RccExt
};

use embedded_graphics::Drawable;


#[entry]
fn main() -> ! {

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
    let clocks: stm32f1xx_hal::rcc::Clocks = rcc.cfgr.use_hse(8.MHz()).freeze(&mut flash.acr);
    let mut gpiob = dp.GPIOB.split();
    let mut delay = cp.SYST.delay(&clocks);


    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
    
    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();


    // Определяем вершины куба в 3D-пространстве
    let mut vertices = [
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ];

    // Определяем рёбра куба (индексы вершин)
    let edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];


    // Функция для проекции 3D-координат на 2D-экран
    fn project(point: [f32; 3], width: f32, height: f32) -> Point {
        let scale = 20.0; // Масштаб
        let x = (point[0] * scale + width / 2.0) as i32;
        let y = (point[1] * scale + height / 2.0) as i32;
        Point::new(x, y)
    }

    // Функция для вращения точки вокруг оси X
    fn rotate_x(point: [f32; 3], angle: f32) -> [f32; 3] {
        let sin = sinf(angle);
        let cos = cosf(angle);
        let y = point[1] * cos - point[2] * sin;
        let z = point[1] * sin + point[2] * cos;
        [point[0], y, z]
    }

    // Функция для вращения точки вокруг оси Y
    fn rotate_y(point: [f32; 3], angle: f32) -> [f32; 3] {
        let sin = sinf(angle);
        let cos = cosf(angle);
        let x = point[0] * cos + point[2] * sin;
        let z = -point[0] * sin + point[2] * cos;
        [x, point[1], z]
    }

    // Функция для вращения точки вокруг оси Z
    fn rotate_z(point: [f32; 3], angle: f32) -> [f32; 3] {
        let sin = sinf(angle);
        let cos = cosf(angle);
        let x = point[0] * cos - point[1] * sin;
        let y = point[0] * sin + point[1] * cos;
        [x, y, point[2]]
    }

    let mut angle_x = 0.0;
    let mut angle_y = 0.0;
    let mut angle_z = 0.0;

    // Фиксированный шаг для углов (в радианах)
    let angle_step = 0.01; // Скорость вращения

    loop {
        display.clear_buffer();

        // Вращение вершин куба
        for vertex in &mut vertices {
            *vertex = rotate_x(*vertex, angle_x);
            *vertex = rotate_y(*vertex, angle_y);
            *vertex = rotate_z(*vertex, angle_z);
        }

        // Отрисовка рёбер куба
        for &(start, end) in &edges {
            let start_point = project(vertices[start], 128.0, 64.0);
            let end_point = project(vertices[end], 128.0, 64.0);

            Line::new(start_point, end_point)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut display)
                .unwrap();
        }

        display.flush().unwrap();

        // Увеличение углов вращения с фиксированным шагом
        angle_x += angle_step;
        angle_y += angle_step;
        angle_z += angle_step;

        // Нормализация углов (приведение к диапазону [0, 2π))
        angle_x = angle_x % (2.0 * core::f32::consts::PI);
        angle_y = angle_y % (2.0 * core::f32::consts::PI);
        angle_z = angle_z % (2.0 * core::f32::consts::PI);

        // info!("angle_x {}", angle_x);

        // // Нормализация углов (чтобы избежать переполнения)
        // if angle_x > 2.0 * core::f32::consts::PI {
        //     angle_x -= 2.0 * core::f32::consts::PI;
        // }
        // if angle_y > 2.0 * core::f32::consts::PI {
        //     angle_y -= 2.0 * core::f32::consts::PI;
        // }
        // if angle_z > 2.0 * core::f32::consts::PI {
        //     angle_z -= 2.0 * core::f32::consts::PI;
        // }

        // Задержка для анимации
        delay.delay_ms(30_u16);
    }

}
