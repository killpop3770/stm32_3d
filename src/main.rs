#![no_std]
#![no_main]
use cortex_m_rt::entry;
use defmt_rtt as _;
use defmt::println;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::{Point as Point2D, Primitive}, primitives::{Line, PrimitiveStyle}};
use libm::{cosf, sinf};
use panic_halt as _;
use ssd1306::{mode::DisplayConfig, prelude::DisplayRotation, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};
use stm32f1xx_hal::{
    flash::FlashExt, gpio::GpioExt, i2c::{BlockingI2c, DutyCycle, Mode}, pac::Peripherals, prelude::*, rcc::RccExt
};

use embedded_graphics::Drawable;

// Простая структура для 3D-точки
struct Point3D {
    x: f32,
    y: f32,
    z: f32,
}

// Проекция 3D-точки на 2D
fn project(point: &Point3D, scale: f32, offset_x: i32, offset_y: i32) -> Point2D {
    let perspective = 200.0 / (200.0 + point.z); // Простая перспектива
    let x = (point.x * perspective * scale) as i32 + offset_x;
    let y = (point.y * perspective * scale) as i32 + offset_y;
    Point2D::new(x, y)
}

// Поворот точки вокруг оси X
fn rotate_x(point: &mut Point3D, angle: f32) {
    let sin_a = sinf(angle);
    let cos_a = cosf(angle);
    let y = point.y * cos_a - point.z * sin_a;
    let z = point.y * sin_a + point.z * cos_a;
    point.y = y;
    point.z = z;
}

// Поворот точки вокруг оси Y
fn rotate_y(point: &mut Point3D, angle: f32) {
    let sin_a = sinf(angle);
    let cos_a = cosf(angle);
    let x = point.x * cos_a - point.z * sin_a;
    let z = point.x * sin_a + point.z * cos_a;
    point.x = x;
    point.z = z;
}

// Функция для вращения точки вокруг оси Z
fn rotate_z(point: &mut Point3D, angle: f32) {
    let sin_a = sinf(angle);
    let cos_a = cosf(angle);
    let x = point.x * cos_a - point.y * sin_a;
    let y = point.x * sin_a + point.y * cos_a;
    point.x = x;
    point.y = y;
}

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
        Point3D {x: -1.0, y: -1.0, z: -1.0},
        Point3D {x: 1.0, y: -1.0, z: -1.0},
        Point3D {x: 1.0, y: 1.0, z: -1.0},
        Point3D {x: -1.0, y: 1.0, z: -1.0},
        Point3D {x: -1.0, y: -1.0, z: 1.0},
        Point3D {x: 1.0, y: -1.0, z: 1.0},
        Point3D {x: 1.0, y: 1.0, z: 1.0},
        Point3D {x: -1.0, y: 1.0, z: 1.0},
    ];

    // Определяем рёбра куба (индексы вершин)
    // Ребра куба
    let edges = [
        (0, 1), (1, 2), (2, 3), (3, 0), // Передняя грань
        (4, 5), (5, 6), (6, 7), (7, 4), // Задняя грань
        (0, 4), (1, 5), (2, 6), (3, 7), // Соединяющие ребра
    ];

    // Фиксированный шаг для углов (в радианах)
    let angle_step = 0.05; // Скорость вращения

    // Проекция и отрисовка ребер
    let scale = 20.0;
    let offset_x = 64; // Центр дисплея по X
    let offset_y = 32; // Центр дисплея по Y

    loop {
        display.clear_buffer();

        // Вращение вершин куба
        for vertex in &mut vertices {
            rotate_x(vertex, angle_step);
            rotate_y(vertex, angle_step);
            rotate_z(vertex, angle_step);
        }

        // Отрисовка рёбер куба
        for &(start, end) in &edges {
            let start_point = project(&vertices[start], scale, offset_x, offset_y);
            let end_point = project(&vertices[end], scale, offset_x, offset_y);

            Line::new(start_point, end_point)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut display)
                .unwrap();
        }

        display.flush().unwrap();
        
        // TODO: Найти причину бесконечного ускорения куба
        //      Сравнить код с и без использования Point3D !!!
        //      Как происходит движение точки вдоль оси X, Y, Z ???
        // TODO: Убрать ребра находящиеся за кубом ??? 

        println!("angle: {}", angle_step);

        // Задержка для анимации
        delay.delay_ms(30_u16);
    }

}
